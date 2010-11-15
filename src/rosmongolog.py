#!/usr/bin/python

###########################################################################
#  rosmongolog.py - Python based ROS to MongoDB logger
#
#  Created: Mon Nov 15 11:30:00 2010
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

import os
import sys
import time
import pprint
import threading
import Queue
from optparse import OptionParser
from datetime import datetime, timedelta

import roslib; roslib.load_manifest(NODE_NAME)
import rospy
import rosgraph.masterapi
import roslib.message
from roslib.rostime import Time, Duration
import rostopic

from pymongo import Connection

class MongoWriter(object):
    def __init__(self, topics, num_threads=10,
                 mongodb_host=None, mongodb_port=None, mongodb_name="roslog"):
        self.subscribers = []
        self.collections = {}
        self.mongoconn = Connection(mongodb_host, mongodb_port)
        self.mongodb = self.mongoconn[mongodb_name]

        collection_names = {}
        for topic in topics:
            if topic and topic[-1] == '/':
                topic = topic[:-1]

            msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic, blocking=True)

            if msg_class is None:
                # occurs on ctrl-C
                raise Exception("Aborted while subscribing to topics")

            sub = rospy.Subscriber(real_topic, msg_class, self.enqueue, topic)
            self.subscribers.append(sub)

            # although the collections is not strictly necessary, since MongoDB could handle
            # pure topic names as collection names and we could then use mongodb[topic], we want
            # to have names that go easier with the query tools, even though there is the theoretical
            # possibility of name classes (hence the check)
            collname = topic.replace("/", "_")[1:]
            if collname in collection_names:
                raise Exception("Two converted topic names clash: %s" % collname)
            self.collections[topic] = self.mongodb[collname]
            collection_names[collname] = True

        self.topics = topics
        self.done = False
        #self.str_fn = roslib.message.strify_message
        self.sep = "\n" #'\033[2J\033[;H'
        self.queue = Queue.Queue()

        self.num_threads = num_threads
        self.write_threads = []
        for i in range(self.num_threads):
            t = threading.Thread(name="MongoWriter "+str(i), target=self.dequeue)
            self.write_threads += [t]
            t.start()


    def message_to_dict(self, val):
        d = {}
        for f in val.__slots__:
            v = getattr(val, f)
            if isinstance(v, rospy.Message):
                d[f] = self.message_to_dict(v)
            elif isinstance(v, Time):
                d[f] = datetime.fromtimestamp(v.secs)
                d[f] += timedelta(microseconds=v.nsecs / 1000.)
            elif isinstance(v, Duration):
                d[f] = v.secs + v.nsecs / 1000000000.
            else:
                d[f] = v
        return d

    def enqueue(self, data, topic, current_time=None):
        self.queue.put((topic, data, current_time))

    def dequeue(self):
        while not self.done:
            t = self.queue.get(True)
            topic = t[0]
            msg   = t[1]
            ctime = t[2]
            if isinstance(t, tuple):
                doc = self.message_to_dict(msg)
                doc["__recorded"] = ctime or datetime.now()
                doc["__topic"]    = topic
                self.collections[topic].insert(doc)

                sys.stdout.write(self.sep + threading.current_thread().getName() + "@" + topic+": ")
                pprint.pprint(doc)
                #sys.stdout.write(self.sep+self.str_fn(data, current_time=current_time) + '\n')

    def shutdown(self):
        self.done = True
        for t in range(self.num_threads):
            self.queue.put("shutdown")

        for t in self.write_threads:
            t.join()


def main(argv):
    parser = OptionParser()
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

    (options, args) = parser.parse_args()

    try:
        rosgraph.masterapi.Master(NODE_NAME).getPid()
    except socket.error:
        print("Failed to communicate with master")

    rospy.init_node(NODE_NAME, anonymous=True)

    mongowriter = MongoWriter(topics=args, num_threads=options.num_threads,
                              mongodb_host=options.mongodb_host,
                              mongodb_port=options.mongodb_port,
                              mongodb_name=options.mongodb_name)

    while not rospy.is_shutdown() and not mongowriter.done:
        time.sleep(0.1)

    mongowriter.shutdown()

if __name__ == "__main__":
    main(sys.argv)
