#!/usr/bin/python

###########################################################################
#  rosmongolog_mp.py - Python based ROS to MongoDB logger (multi-process)
#
#  Created: Sun Dec 05 19:45:51 2010
#  Copyright  2010  Tim Niemueller [www.niemueller.de]
#                   Carnegie Mellon University
#                   Intel Labs Pittsburgh
###########################################################################

#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE.GPL file in the doc directory.

# make sure we aren't using floor division
from __future__ import division, with_statement

NODE_NAME='rosmongolog'
WORKER_NODE_NAME = "rosmongolog_worker_%d_%s"
QUEUE_MAXSIZE = 100

import roslib; roslib.load_manifest(NODE_NAME)

import os
import re
import sys
import time
import pprint
import string
import subprocess
from threading import Thread, Timer
from multiprocessing import Process, Lock, Condition, Queue, Value, current_process
from Queue import Empty
from optparse import OptionParser
from datetime import datetime, timedelta
from time import sleep
from random import randint
from tf.msg import tfMessage
from sensor_msgs.msg import PointCloud, CompressedImage
from rviz_intel.msg import TriangleMesh

from setproctitle import setproctitle

import rospy
import rosgraph.masterapi
import roslib.message
from roslib.rostime import Time, Duration
import rostopic
import rrdtool

from pymongo import Connection, SLOW_ONLY
from pymongo.errors import InvalidDocument, InvalidStringData

import rrdtool

BACKLOG_WARN_LIMIT = 100
STATS_LOOPTIME     = 10
STATS_GRAPHTIME    = 60

class Counter(object):
    def __init__(self, value = None, lock = True):
        self.count = value or Value('i', 0, lock=lock)
        self.mutex = Lock()

    def increment(self, by = 1):
        with self.mutex: self.count.value += by

    def value(self):
        with self.mutex: return self.count.value

class Barrier(object):
    def __init__(self, num_threads):
        self.num_threads = num_threads
        self.threads_left = Value('i', num_threads, lock=True)
        self.mutex = Lock()
        self.waitcond = Condition(self.mutex)

    def wait(self):
        self.mutex.acquire()
        self.threads_left.value -= 1
        if self.threads_left.value == 0:
            self.threads_left.value = self.num_threads
            self.waitcond.notify_all()
            self.mutex.release()
        else:
            self.waitcond.wait()
            self.mutex.release()


class WorkerProcess(object):
    def __init__(self, idnum, topic, collname, in_counter_value, out_counter_value,
                 drop_counter_value, queue_maxsize,
                 mongodb_host, mongodb_port, mongodb_name):
        self.name = "WorkerProcess-%4d-%s" % (idnum, topic)
        self.id = idnum
        self.topic = topic
        self.collname = collname
        self.queue = Queue(queue_maxsize)
        self.out_counter = Counter(out_counter_value)
        self.in_counter  = Counter(in_counter_value)
        self.drop_counter = Counter(drop_counter_value)
        self.worker_out_counter = Counter()
        self.worker_in_counter  = Counter()
        self.worker_drop_counter = Counter()
        self.mongodb_host = mongodb_host
        self.mongodb_port = mongodb_port
        self.mongodb_name = mongodb_name
        self.quit = False

        self.process = Process(name=self.name, target=self.run)
        self.process.start()

    def init(self):
        setproctitle("rosmongolog_mp %s" % self.topic)

        self.mongoconn = Connection(self.mongodb_host, self.mongodb_port)
        self.mongodb = self.mongoconn[self.mongodb_name]
        self.mongodb.set_profiling_level = SLOW_ONLY

        self.collection = self.mongodb[self.collname]
        self.collection.count()

        rospy.init_node(WORKER_NODE_NAME % (self.id, self.collname), anonymous=False)

        self.subscriber = None
        while not self.subscriber:
            try:
                msg_class, real_topic, msg_eval = rostopic.get_topic_class(self.topic, blocking=True)
                self.subscriber = rospy.Subscriber(real_topic, msg_class, self.enqueue, self.topic)
            except rostopic.ROSTopicIOException:
                print("FAILED to subscribe, will keep trying %s" % self.name)
                time.sleep(randint(1,10))
            except rospy.ROSInitException:
                print("FAILED to initialize, will keep trying %s" % self.name)
                time.sleep(randint(1,10))
                self.subscriber = None

    def run(self):
        self.init()

        print("ACTIVE: %s" % self.name)

        # run the thread
        self.dequeue()

        # free connection
        # self.mongoconn.end_request()

    def shutdown(self):
        self.quit = True
        self.queue.put("shutdown")
        self.process.join()
        self.process.terminate()
        rospy.signal_shutdown("shutdown")

    def sanitize_value(self, v):
        if isinstance(v, rospy.Message):
            return self.message_to_dict(v)
        elif isinstance(v, Time):
            t = datetime.fromtimestamp(v.secs)
            return t + timedelta(microseconds=v.nsecs / 1000.)
        elif isinstance(v, Duration):
            return v.secs + v.nsecs / 1000000000.
        elif isinstance(v, list):
            return [self.sanitize_value(t) for t in v]
        else:
            return v

    def message_to_dict(self, val):
        d = {}
        for f in val.__slots__:
            d[f] = self.sanitize_value(getattr(val, f))
        return d

    def qsize(self):
        return self.queue.qsize()

    def enqueue(self, data, topic, current_time=None):
        if not self.quit:
            if self.queue.full():
                try:
                    self.queue.get_nowait()
                    self.drop_counter.increment()
                    self.worker_drop_counter.increment()
                except Empty:
                    pass
            self.queue.put((topic, data, current_time or datetime.now()))
            self.in_counter.increment()
            self.worker_in_counter.increment()

    def dequeue(self):
        while not self.quit:
            t = None
            try:
                t = self.queue.get(True)
            except IOError:
                # Anticipate Ctrl-C
                print("Quit W1")
                self.quit = True
                return
            if isinstance(t, tuple):
                self.out_counter.increment()
                self.worker_out_counter.increment()
                topic = t[0]
                msg   = t[1]
                ctime = t[2]

                if isinstance(msg, rospy.Message):
                    doc = self.message_to_dict(msg)
                    doc["__recorded"] = ctime or datetime.now()
                    doc["__topic"]    = topic
                    try:
                        #print(self.sep + threading.current_thread().getName() + "@" + topic+": ")
                        #pprint.pprint(doc)
                        self.collection.insert(doc)
                    except InvalidDocument, e:
                        print("InvalidDocument " + current_process().name + "@" + topic +": \n")
                        print e
                    except InvalidStringData, e:
                        print("InvalidStringData " + current_process().name + "@" + topic +": \n")
                        print e

            else:
                print("Quit W2")
                self.quit = True
        print("Quit W3")


class SubprocessWorker(object):
    def __init__(self, idnum, topic, collname, in_counter_value, out_counter_value,
                 drop_counter_value, queue_maxsize,
                 mongodb_host, mongodb_port, mongodb_name, cpp_logger):

        self.name = "SubprocessWorker-%4d-%s" % (idnum, topic)
        self.id = idnum
        self.topic = topic
        self.collname = collname
        self.queue = Queue(queue_maxsize)
        self.out_counter = Counter(out_counter_value)
        self.in_counter  = Counter(in_counter_value)
        self.drop_counter = Counter(drop_counter_value)
        self.worker_out_counter = Counter()
        self.worker_in_counter  = Counter()
        self.worker_drop_counter = Counter()
        self.mongodb_host = mongodb_host
        self.mongodb_port = mongodb_port
        self.mongodb_name = mongodb_name
        self.quit = False
        self.qsize = 0

        self.thread = Thread(name=self.name, target=self.run)

        mongodb_host_port = "%s:%d" % (mongodb_host, mongodb_port)
        collection = "%s.%s" % (mongodb_name, collname)
        nodename = WORKER_NODE_NAME % (self.id, self.collname)
        self.process = subprocess.Popen([cpp_logger, "-t", topic, "-n", nodename,
                                         "-m", mongodb_host_port, "-c", collection],
                                        stdout=subprocess.PIPE)

        self.thread.start()

    def qsize(self):
        return self.qsize

    def run(self):
        while not self.quit:
            line = self.process.stdout.readline().rstrip()
            if line == "": continue
            arr = string.split(line, ":")
            self.in_counter.increment(int(arr[0]))
            self.out_counter.increment(int(arr[1]))
            self.drop_counter.increment(int(arr[2]))
            self.qsize = int(arr[3])

            self.worker_in_counter.increment(int(arr[0]))
            self.worker_out_counter.increment(int(arr[1]))
            self.worker_drop_counter.increment(int(arr[2]))

    def shutdown(self):
        self.quit = True
        self.process.kill()
        self.process.wait()

class MongoWriter(object):
    def __init__(self, topics = [], graph_topics = False,
                 graph_dir = ".", graph_clear = False,
                 all_topics = False, all_topics_interval = 5,
                 exclude_topics = [],
                 mongodb_host=None, mongodb_port=None, mongodb_name="roslog"):
        self.graph_dir = graph_dir
        self.graph_topics = graph_topics
        self.graph_clear = graph_clear
        self.all_topics = all_topics
        self.all_topics_interval = all_topics_interval
        self.exclude_topics = exclude_topics
        self.mongodb_host = mongodb_host
        self.mongodb_port = mongodb_port
        self.mongodb_name = mongodb_name
        self.quit = False
        self.topics = set()
        #self.str_fn = roslib.message.strify_message
        self.sep = "\n" #'\033[2J\033[;H'
        self.in_counter = Counter()
        self.out_counter = Counter()
        self.drop_counter = Counter()
        self.workers = {}


        setproctitle("rosmongolog_mp MAIN")

        self.exclude_regex = []
        for et in self.exclude_topics:
            self.exclude_regex.append(re.compile(et))
        self.exclude_already = []

        self.init_rrd()

        self.subscribe_topics(set(topics))
        if self.all_topics:
            print("All topics")
            self.ros_master = rosgraph.masterapi.Master(NODE_NAME)
            self.update_topics(restart=False)
        rospy.init_node(NODE_NAME, anonymous=True)

        self.start_all_topics_timer()

    def subscribe_topics(self, topics):
        for topic in topics:
            if topic and topic[-1] == '/':
                topic = topic[:-1]

            if topic in self.topics: continue
            if topic in self.exclude_already: continue

            do_continue = False
            for tre in self.exclude_regex:
                if tre.match(topic):
                    print("*** IGNORING topic %s due to exclusion rule" % topic)
                    do_continue = True
                    self.exclude_already.append(topic)
                    break
            if do_continue: continue

            # although the collections is not strictly necessary, since MongoDB could handle
            # pure topic names as collection names and we could then use mongodb[topic], we want
            # to have names that go easier with the query tools, even though there is the theoretical
            # possibility of name classes (hence the check)
            collname = topic.replace("/", "_")[1:]
            if collname in self.workers.keys():
                print("Two converted topic names clash: %s, ignoring topic %s"
                      % (collname, topic))
            else:
                print("Adding topic %s" % topic)
                self.workers[collname] = self.create_worker(len(self.workers), topic, collname)
                self.topics |= set([topic])

    def create_worker(self, idnum, topic, collname):
        msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic, blocking=True)

        w = None
        if msg_class == tfMessage:
            print("DETECTED transform topic %s, using fast C++ logger" % topic)
            w = SubprocessWorker(idnum, topic, collname,
                                 self.in_counter.count, self.out_counter.count,
                                 self.drop_counter.count, QUEUE_MAXSIZE,
                                self.mongodb_host, self.mongodb_port, self.mongodb_name,
                                 "./rosmongolog_tf")
        elif msg_class == PointCloud:
            print("DETECTED point cloud topic %s, using fast C++ logger" % topic)
            w = SubprocessWorker(idnum, topic, collname,
                                 self.in_counter.count, self.out_counter.count,
                                 self.drop_counter.count, QUEUE_MAXSIZE,
                                 self.mongodb_host, self.mongodb_port, self.mongodb_name,
                                 "./rosmongolog_pcl")
        elif msg_class == CompressedImage:
            print("DETECTED compressed image topic %s, using fast C++ logger" % topic)
            w = SubprocessWorker(idnum, topic, collname,
                                 self.in_counter.count, self.out_counter.count,
                                 self.drop_counter.count, QUEUE_MAXSIZE,
                                 self.mongodb_host, self.mongodb_port, self.mongodb_name,
                                 "./rosmongolog_cimg")
        elif msg_class == TriangleMesh:
            print("DETECTED triangle mesh topic %s, using fast C++ logger" % topic)
            w = SubprocessWorker(idnum, topic, collname,
                                 self.in_counter.count, self.out_counter.count,
                                 self.drop_counter.count, QUEUE_MAXSIZE,
                                 self.mongodb_host, self.mongodb_port, self.mongodb_name,
                                 "./rosmongolog_trimesh")
        else:
            w = WorkerProcess(idnum, topic, collname,
                              self.in_counter.count, self.out_counter.count,
                              self.drop_counter.count, QUEUE_MAXSIZE,
                              self.mongodb_host, self.mongodb_port, self.mongodb_name)

        if self.graph_topics: self.assert_worker_rrd(collname)
        
        return w


    def run(self):
        looping_threshold  = timedelta(0, STATS_LOOPTIME,  0)
        graphing_threshold = timedelta(0, STATS_GRAPHTIME - STATS_GRAPHTIME*0.01, 0)
        graphing_last      = datetime.now()

        while not rospy.is_shutdown() and not self.quit:
            started = datetime.now()

            self.update_rrd()

            if datetime.now() - graphing_last >= graphing_threshold:
                self.graph_rrd()
                graphing_last = datetime.now()

            # the following code makes sure we run once per STATS_LOOPTIME, taking
            # varying run-times and interrupted sleeps into account
            td = datetime.now() - started
            while not rospy.is_shutdown() and not self.quit and td < looping_threshold:
                sleeptime = STATS_LOOPTIME - (td.microseconds + (td.seconds + td.days * 24 * 3600) * 10**6) / 10**6
                if sleeptime > 0:
                    sleep(sleeptime)
                td = datetime.now() - started

        print("Quit M1")


    def shutdown(self):
        self.quit = True
        if hasattr(self, "all_topics_timer"): self.all_topics_timer.cancel()
        print("Quit M2")
        for name, w in self.workers.items():
            print("Shutdown %s %d" % (w, os.getpid()))
            w.shutdown()

    def start_all_topics_timer(self):
        if not self.all_topics or self.quit: return
        self.all_topics_timer = Timer(self.all_topics_interval, self.update_topics)
        self.all_topics_timer.start()


    def update_topics(self, restart=True):
        if not self.all_topics or self.quit: return
        ts = self.ros_master.getPublishedTopics("/")
        topics = set([t for t, t_type in ts if t != "/rosout" and t != "/rosout_agg"])
        new_topics = topics - self.topics
        self.subscribe_topics(new_topics)
        if restart: self.start_all_topics_timer()

    def get_memory_usage_for_pid(self, pid):

        scale = {'kB': 1024, 'mB': 1024 * 1024,
                 'KB': 1024, 'MB': 1024 * 1024}
        try:
            f = open("/proc/%d/status" % pid)
            t = f.read()
            f.close()
        except:
            return (0, 0, 0)

        if t == "": return (0, 0, 0)

        try:
            tmp   = t[t.index("VmSize:"):].split(None, 3)
            size  = int(tmp[1]) * scale[tmp[2]]
            tmp   = t[t.index("VmRSS:"):].split(None, 3)
            rss   = int(tmp[1]) * scale[tmp[2]]
            tmp   = t[t.index("VmStk:"):].split(None, 3)
            stack = int(tmp[1]) * scale[tmp[2]]
            return (size, rss, stack)
        except ValueError:
            return (0, 0, 0)

    def get_memory_usage(self):
        size, rss, stack = 0, 0, 0
        for _, w in self.workers.items():
            pmem = self.get_memory_usage_for_pid(w.process.pid)
            size  += pmem[0]
            rss   += pmem[1]
            stack += pmem[2]
        #print("Size: %d  RSS: %s  Stack: %s" % (size, rss, stack))
        return (size, rss, stack)

    def assert_rrd(self, file, *data_sources):
        if not os.path.isfile(file) or self.graph_clear:
            rrdtool.create(file, "--step", "10", "--start", "0",
                           # remember that we always need to add the previous RRA time range
                           # hence number of rows is not directly calculated by desired time frame
                           "RRA:AVERAGE:0.5:1:720",    #  2 hours of 10 sec  averages
                           "RRA:AVERAGE:0.5:3:1680",   # 12 hours of 30 sec  averages
                           "RRA:AVERAGE:0.5:30:456",   #  1 day   of  5 min  averages
                           "RRA:AVERAGE:0.5:180:412",  #  7 days  of 30 min  averages
                           "RRA:AVERAGE:0.5:720:439",  #  4 weeks of  2 hour averages
                           "RRA:AVERAGE:0.5:8640:402", #  1 year  of  1 day averages
                           "RRA:MIN:0.5:1:720",
                           "RRA:MIN:0.5:3:1680",
                           "RRA:MIN:0.5:30:456",
                           "RRA:MIN:0.5:180:412",
                           "RRA:MIN:0.5:720:439",
                           "RRA:MIN:0.5:8640:402",
                           "RRA:MAX:0.5:1:720",
                           "RRA:MAX:0.5:3:1680",
                           "RRA:MAX:0.5:30:456",
                           "RRA:MAX:0.5:180:412",
                           "RRA:MAX:0.5:720:439",
                           "RRA:MAX:0.5:8640:402",
                           *data_sources)

    def graph_rrd(self):
        #print("Generating graphs")
        time_started = datetime.now()
        rrdtool.graph("%s/logstats.png" % self.graph_dir,
                      "--start=-600", "--end=-10",
                      "--disable-rrdtool-tag", "--width=560",
                      "--font", "LEGEND:10:", "--font", "UNIT:8:",
                      "--font", "TITLE:12:", "--font", "AXIS:8:",
                      "--title=MongoDB Logging Stats",
                      "--vertical-label=messages/sec",
                      "--slope-mode",
                      "DEF:qsize=%s/logstats.rrd:qsize:AVERAGE:step=10" % self.graph_dir,
                      "DEF:in=%s/logstats.rrd:in:AVERAGE:step=10" % self.graph_dir,
                      "DEF:out=%s/logstats.rrd:out:AVERAGE:step=10" % self.graph_dir,
                      "DEF:drop=%s/logstats.rrd:drop:AVERAGE:step=10" % self.graph_dir,
                      "LINE1:qsize#FF7200:Queue Size",
                      "GPRINT:qsize:LAST:Current\\:%8.2lf %s",
                      "GPRINT:qsize:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:qsize:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:in#503001:In",
                      "GPRINT:in:LAST:        Current\\:%8.2lf %s",
                      "GPRINT:in:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:in:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:out#EDAC00:Out",
                      "GPRINT:out:LAST:       Current\\:%8.2lf %s",
                      "GPRINT:out:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:out:MAX:Maximum\\:%8.2lf %s\\n",
                      "LINE1:drop#506101:Dropped",
                      "GPRINT:drop:LAST:   Current\\:%8.2lf %s",
                      "GPRINT:drop:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:drop:MAX:Maximum\\:%8.2lf %s\\n")

        if self.graph_topics:
            for _, w in self.workers.items():
                #worker_time_started = datetime.now()
                rrdtool.graph("%s/%s.png" % (self.graph_dir, w.collname),
                              "--start=-600", "--end=-10",
                              "--disable-rrdtool-tag", "--width=560",
                              "--font", "LEGEND:10:", "--font", "UNIT:8:",
                              "--font", "TITLE:12:", "--font", "AXIS:8:",
                              "--title=%s" % w.topic,
                              "--vertical-label=messages/sec",
                              "--slope-mode",
                              "DEF:qsize=%s/%s.rrd:qsize:AVERAGE:step=10" % (self.graph_dir, w.collname),
                              "DEF:in=%s/%s.rrd:in:AVERAGE:step=10" % (self.graph_dir, w.collname),
                              "DEF:out=%s/%s.rrd:out:AVERAGE:step=10" % (self.graph_dir, w.collname),
                              "DEF:drop=%s/%s.rrd:drop:AVERAGE:step=10" % (self.graph_dir, w.collname),
                              "LINE1:qsize#FF7200:Queue Size",
                              "GPRINT:qsize:LAST:Current\\:%8.2lf %s",
                              "GPRINT:qsize:AVERAGE:Average\\:%8.2lf %s",
                              "GPRINT:qsize:MAX:Maximum\\:%8.2lf %s\\n",
                              "LINE1:in#503001:In",
                              "GPRINT:in:LAST:        Current\\:%8.2lf %s",
                              "GPRINT:in:AVERAGE:Average\\:%8.2lf %s",
                              "GPRINT:in:MAX:Maximum\\:%8.2lf %s\\n",
                              "LINE1:out#EDAC00:Out",
                              "GPRINT:out:LAST:       Current\\:%8.2lf %s",
                              "GPRINT:out:AVERAGE:Average\\:%8.2lf %s",
                              "GPRINT:out:MAX:Maximum\\:%8.2lf %s\\n",
                              "LINE1:drop#506101:Dropped",
                              "GPRINT:drop:LAST:   Current\\:%8.2lf %s",
                              "GPRINT:drop:AVERAGE:Average\\:%8.2lf %s",
                              "GPRINT:drop:MAX:Maximum\\:%8.2lf %s\\n")

                #worker_time_elapsed = datetime.now() - worker_time_started
                #print("Generated worker graph for %s, took %s" % (w.topic, worker_time_elapsed))


        rrdtool.graph("%s/logmemory.png" % self.graph_dir,
                      "--start=-600", "--end=-10",
                      "--disable-rrdtool-tag", "--width=560",
                      "--font", "LEGEND:10:", "--font", "UNIT:8:",
                      "--font", "TITLE:12:", "--font", "AXIS:8:",
                      "--title=ROS MongoLog Memory Usage",
                      "--vertical-label=bytes",
                      "--slope-mode",
                      "DEF:size=%s/logmemory.rrd:size:AVERAGE:step=10" % self.graph_dir,
                      "DEF:rss=%s/logmemory.rrd:rss:AVERAGE:step=10" % self.graph_dir,
                      "AREA:size#FF7200:Total",
                      "GPRINT:size:LAST:   Current\\:%8.2lf %s",
                      "GPRINT:size:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:size:MAX:Maximum\\:%8.2lf %s\\n",
                      "AREA:rss#503001:Resident",
                      "GPRINT:rss:LAST:Current\\:%8.2lf %s",
                      "GPRINT:rss:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:rss:MAX:Maximum\\:%8.2lf %s\\n")
        time_elapsed = datetime.now() - time_started
        print("Generated graphs, took %s" % time_elapsed)

    def init_rrd(self):
        self.assert_rrd("%s/logstats.rrd" % self.graph_dir,
                        "DS:qsize:GAUGE:30:0:U",
                        "DS:in:COUNTER:30:0:U",
                        "DS:out:COUNTER:30:0:U",
                        "DS:drop:COUNTER:30:0:U")

        self.assert_rrd("%s/logmemory.rrd" % self.graph_dir,
                        "DS:size:GAUGE:30:0:U",
                        "DS:rss:GAUGE:30:0:U",
                        "DS:stack:GAUGE:30:0:U")

    def assert_worker_rrd(self, collname):
        self.assert_rrd("%s/%s.rrd" % (self.graph_dir, collname),
                        "DS:qsize:GAUGE:30:0:U",
                        "DS:in:COUNTER:30:0:U",
                        "DS:out:COUNTER:30:0:U",
                        "DS:drop:COUNTER:30:0:U")


    def update_rrd(self):
        # we do not lock here, we are not interested in super-precise
        # values for this, but we do care for high performance processing
        qsize = 0
        #print("Updating graphs")
        time_started = datetime.now()
        for _, w in self.workers.items():
            wqsize = w.queue.qsize()
            qsize += wqsize
            if wqsize > QUEUE_MAXSIZE/2: print("Excessive queue size %6d: %s" % (wqsize, w.name))

            if self.graph_topics:
                rrdtool.update("%s/%s.rrd" % (self.graph_dir, w.collname), "N:%d:%d:%d:%d" %
                               (wqsize, w.worker_in_counter.count.value,
                                w.worker_out_counter.count.value,
                                w.worker_drop_counter.count.value))

        rrdtool.update("%s/logstats.rrd" % self.graph_dir, "N:%d:%d:%d:%d" %
                       (qsize, self.in_counter.count.value, self.out_counter.count.value,
                        self.drop_counter.count.value))
        rrdtool.update("%s/logmemory.rrd" % self.graph_dir, "N:%d:%d:%d" % self.get_memory_usage())

        time_elapsed = datetime.now() - time_started
        print("Updated graphs, total queue size %d, dropped %d, took %s" %
              (qsize, self.drop_counter.count.value, time_elapsed))


def main(argv):
    parser = OptionParser()
    parser.usage += " [TOPICs...]"
    parser.add_option("--mongodb-host", dest="mongodb_host",
                      help="Hostname of MongoDB", metavar="HOST",
                      default="localhost")
    parser.add_option("--mongodb-port", dest="mongodb_port",
                      help="Hostname of MongoDB", type="int",
                      metavar="PORT", default=27017)
    parser.add_option("--mongodb-name", dest="mongodb_name",
                      help="Name of DB in which to store values",
                      metavar="NAME", default="roslog")
    parser.add_option("-a", "--all-topics", dest="all_topics", default=False,
                      action="store_true",
                      help="Log all existing topics (still excludes /rosout, /rosout_agg)")
    parser.add_option("--all-topics-interval", dest="all_topics_interval", default=5,
                      help="Time in seconds between checks for new topics", type="int")
    parser.add_option("-x", "--exclude", dest="exclude",
                      help="Exclude topics matching REGEX, may be given multiple times",
                      action="append", type="string", metavar="REGEX", default=[])
    parser.add_option("--graph-topics", dest="graph_topics", default=False,
                      action="store_true",
                      help="Write graphs per topic")
    parser.add_option("--graph-clear", dest="graph_clear", default=False,
                      action="store_true",
                      help="Remove existing RRD files.")
    parser.add_option("--graph-dir", dest="graph_dir", default=".",
                      help="Directory in which to create the graphs")
    (options, args) = parser.parse_args()

    if not options.all_topics and len(args) == 0:
        parser.print_help()
        return

    try:
        rosgraph.masterapi.Master(NODE_NAME).getPid()
    except socket.error:
        print("Failed to communicate with master")

    mongowriter = MongoWriter(topics=args, graph_topics = options.graph_topics,
                              graph_dir = options.graph_dir,
                              graph_clear = options.graph_clear,
                              all_topics=options.all_topics,
                              all_topics_interval = options.all_topics_interval,
                              exclude_topics = options.exclude,
                              mongodb_host=options.mongodb_host,
                              mongodb_port=options.mongodb_port,
                              mongodb_name=options.mongodb_name)

    mongowriter.run()
    mongowriter.shutdown()

if __name__ == "__main__":
    main(sys.argv)
