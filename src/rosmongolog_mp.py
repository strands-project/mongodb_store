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
WORKER_NODE_NAME = "rosmongolog_worker_%d"

import os
import re
import sys
import time
import pprint
from threading import Timer
from Queue import Queue
from multiprocessing import Process, Lock, Condition, Value, current_process
from optparse import OptionParser
from datetime import datetime, timedelta
from time import sleep

import roslib; roslib.load_manifest(NODE_NAME)
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
    def __init__(self, value = None):
        self.count = value or Value('i', 0)
        self.mutex = Lock()

    def increment(self):
        with self.mutex: self.count.value += 1

    def value(self):
        with self.mutex: return self.count.value

class Barrier(object):
    def __init__(self, num_threads):
        self.num_threads = num_threads
        self.threads_left = Value('i', num_threads)
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
                 mongodb_host, mongodb_port, mongodb_name):
        self.name = "WorkerProcess %d" % idnum
        self.id = idnum
        self.topic = topic
        self.collname = collname
        self.queue = Queue()
        self.out_counter = Counter(out_counter_value)
        self.in_counter  = Counter(in_counter_value)
        self.mongodb_host = mongodb_host
        self.mongodb_port = mongodb_port
        self.mongodb_name = mongodb_name
        self.quit = False

        self.process = Process(name=self.name, target=self.run)
        self.process.start()

    def init(self):
        self.mongoconn = Connection(self.mongodb_host, self.mongodb_port)
        self.mongodb = self.mongoconn[self.mongodb_name]
        self.mongodb.set_profiling_level = SLOW_ONLY

        self.collection = self.mongodb[self.collname]
        self.collection.count()

        rospy.init_node(WORKER_NODE_NAME % self.id, anonymous=True)

        msg_class, real_topic, msg_eval = rostopic.get_topic_class(self.topic, blocking=True)
        self.subscriber = rospy.Subscriber(real_topic, msg_class, self.enqueue, self.topic)

    def run(self):
        self.init()

        # run the thread
        self.dequeue()

        # free connection
        self.mongoconn.end_request()

    def quit_watch(self):
        self.quit_queue.get(True)
        self.shutdown()

    def shutdown(self):
        self.quit = True
        self.queue.put("shutdown")
        self.process.join(1)
        self.process.terminate()

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

    def enqueue(self, data, topic, current_time=None):
        if not self.quit:
            self.queue.put((topic, data, current_time or datetime.now()))
            self.in_counter.increment()

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


class MongoWriter(object):
    def __init__(self, topics = [], num_threads=10,
                 all_topics = False, all_topics_interval = 5,
                 exclude_topics = [],
                 mongodb_host=None, mongodb_port=None, mongodb_name="roslog"):
        self.num_threads = num_threads
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
        self.queue = Queue()
        self.in_counter = Counter()
        self.out_counter = Counter()
        self.workers = {}

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
        print("Initialization complete, logging enabled")

    def subscribe_topics(self, topics):
        for topic in topics:
            if topic and topic[-1] == '/':
                topic = topic[:-1]

            if topic in self.topics: continue
            if topic in self.exclude_already: continue

            do_continue = False
            for tre in self.exclude_regex:
                if tre.match(topic):
                    print("Ignoring topic %s due to exclusion rule" % topic)
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
                #self.collections[topic] = self.mongodb[collname]
                w = WorkerProcess(len(self.workers), topic, collname,
                                  self.in_counter.count, self.out_counter.count,
                                  self.mongodb_host, self.mongodb_port, self.mongodb_name)
                self.workers[collname] = w
                self.topics |= set([topic])

    def run(self):
        looping_threshold  = timedelta(0, STATS_LOOPTIME,  0)
        graphing_threshold = timedelta(0, STATS_GRAPHTIME, 0)
        graphing_last      = datetime.now()

        while not rospy.is_shutdown() and not self.quit:
            started = datetime.now()

            self.update_rrd()

            if datetime.now() - graphing_last > graphing_threshold:
                print("Generating graphs, queue size %d" % self.queue.qsize())
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

    def get_memory_usage(self):
        scale = {'kB': 1024, 'mB': 1024 * 1024,
                 'KB': 1024, 'MB': 1024 * 1024}
        try:
            f = open("/proc/%d/status" % os.getpid())
            t = f.read()
            f.close()
        except:
            return (0, 0, 0)

        tmp   = t[t.index("VmSize:"):].split(None, 3)
        size  = int(tmp[1]) * scale[tmp[2]]
        tmp   = t[t.index("VmRSS:"):].split(None, 3)
        rss   = int(tmp[1]) * scale[tmp[2]]
        tmp   = t[t.index("VmStk:"):].split(None, 3)
        stack = int(tmp[1]) * scale[tmp[2]]
        return (size, rss, stack)

    def create_rrd(self, file, *data_sources):
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
        rrdtool.graph("logstats.png",
                      "--start=-3600", "--end=-10",
                      "--disable-rrdtool-tag", "--width=560",
                      "--font", "LEGEND:10:", "--font", "UNIT:8:",
                      "--font", "TITLE:12:", "--font", "AXIS:8:",
                      "--title=MongoDB Logging Stats",
                      "DEF:qsize=logstats.rrd:qsize:AVERAGE:step=10",
                      "DEF:in=logstats.rrd:in:AVERAGE:step=10",
                      "DEF:out=logstats.rrd:out:AVERAGE:step=10",
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
                      "GPRINT:out:MAX:Maximum\\:%8.2lf %s\\n")


        rrdtool.graph("logmemory.png",
                      "--start=-3600", "--end=-10",
                      "--disable-rrdtool-tag", "--width=560",
                      "--font", "LEGEND:10:", "--font", "UNIT:8:",
                      "--font", "TITLE:12:", "--font", "AXIS:8:",
                      "--title=ROS MongoLog Memory Usage",
                      "DEF:size=logmemory.rrd:size:AVERAGE:step=10",
                      "DEF:rss=logmemory.rrd:rss:AVERAGE:step=10",
                      "AREA:size#FF7200:Total",
                      "GPRINT:size:LAST:   Current\\:%8.2lf %s",
                      "GPRINT:size:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:size:MAX:Maximum\\:%8.2lf %s\\n",
                      "AREA:rss#503001:Resident",
                      "GPRINT:rss:LAST:Current\\:%8.2lf %s",
                      "GPRINT:rss:AVERAGE:Average\\:%8.2lf %s",
                      "GPRINT:rss:MAX:Maximum\\:%8.2lf %s\\n")

    def init_rrd(self):
        self.create_rrd("logstats.rrd",
                        "DS:qsize:GAUGE:30:0:U",
                        "DS:in:COUNTER:30:0:U",
                        "DS:out:COUNTER:30:0:U")

        self.create_rrd("logmemory.rrd",
                        "DS:size:GAUGE:30:0:U",
                        "DS:rss:GAUGE:30:0:U",
                        "DS:stack:GAUGE:30:0:U")

    def update_rrd(self):
        # we do not lock here, we are not interested in super-precise
        # values for this, but we do care for high performance processing
        rrdtool.update("logstats.rrd", "N:%d:%d:%d" %
                       (self.queue.qsize(), self.in_counter.count.value, self.out_counter.count.value))
        rrdtool.update("logmemory.rrd", "N:%d:%d:%d" % self.get_memory_usage())


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
    parser.add_option("--num-threads", dest="num_threads",
                      help="Number of writer threads", type="int",
                      metavar="N", default=10)
    parser.add_option("-a", "--all-topics", dest="all_topics", default=False,
                      action="store_true",
                      help="Log all existing topics (still excludes /rosout, /rosout_agg)")
    parser.add_option("--all-topics-interval", dest="all_topics_interval", default=5,
                      help="Time in seconds between checks for new topics", type="int")
    parser.add_option("-x", "--exclude", dest="exclude",
                      help="Exclude topics matching REGEX, may be given multiple times",
                      action="append", type="string", metavar="REGEX", default=[])
    (options, args) = parser.parse_args()

    if not options.all_topics and len(args) == 0:
        parser.print_help()
        return

    try:
        rosgraph.masterapi.Master(NODE_NAME).getPid()
    except socket.error:
        print("Failed to communicate with master")

    mongowriter = MongoWriter(topics=args, num_threads=options.num_threads,
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
