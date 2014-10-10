#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store.util as mg_util
import sys
import time
import pymongo
from multiprocessing import Process
import calendar
import datetime
import threading
import multiprocessing 
from rosgraph_msgs.msg import Clock
import signal

MongoClient = mg_util.import_MongoClient()

TIME_KEY = '_meta.inserted_at'

def max_time(collection):
    return collection.find_one(sort=[(TIME_KEY, pymongo.DESCENDING)])['_meta']['inserted_at']

def min_time(collection):
    return collection.find_one(sort=[(TIME_KEY, pymongo.ASCENDING)])['_meta']['inserted_at']

def to_ros_time(dt):
    return rospy.Time(calendar.timegm(dt.utctimetuple()), dt.microsecond * 1000)

def to_datetime(rt):
    return datetime.datetime.utcfromtimestamp(rt.secs) + datetime.timedelta(microseconds     = rt.nsecs / 1000)

def ros_time_strftime(rt, format):
    """ converts a ros time to a datetime and calls strftime on it with the given format """
    return to_datetime(rt).strftime(format)


class PlayerProcess(object):
    
    def __init__(self, event, start_time, end_time):
        super(PlayerProcess, self).__init__()

        self.event = event

        self.start_time = start_time
        self.end_time = end_time


        self.running = multiprocessing.Value('b', True)
        self.player_process = multiprocessing.Process(target=self.run, args=[self.running])
        
    def start(self):        
        self.player_process.start()

    def stop(self):        
        self.running.value = False

    def join(self):        
        self.player_process.join()

    def is_running(self):
        return self.running.value

class TopicPlayer(PlayerProcess):
    """ """
    def __init__(self, mongodb_host, mongodb_port, db_name, collection_name, event, start_time, end_time):
        super(TopicPlayer, self).__init__(event, start_time, end_time)
        
        self.mongodb_host = mongodb_host
        self.mongodb_port = mongodb_port
        self.db_name = db_name
        self.collection_name = collection_name


    def init(self):
        """ Called in subprocess to do process-specific initialisation """

        rospy.init_node("mongodb_playback_%s" % self.collection_name) 

        # clear signal handlers in this child process, rospy will handle signals for us
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.mongo_client=MongoClient(self.mongodb_host, self.mongodb_port)
        self.collection = self.mongo_client[self.db_name][self.collection_name]


    def run(self, running):

        self.init()

        rospy.loginfo('Topic playback ready %s %s' % (self.collection.name, rospy.get_param('use_sim_time')))

        while running.value:
            rospy.loginfo('%s %s' % (self.collection.name, rospy.get_param('use_sim_time')))
            rospy.sleep(1)

        self.mongo_client.disconnect()
        rospy.loginfo('Topic playback finished %s' % self.collection.name)            



class ClockPlayer(PlayerProcess):
    """ Plays a clock message in a separate thread"""
    def __init__(self, event, start_time, end_time, pre_roll = rospy.Duration(0), post_roll = rospy.Duration(0)):
        super(ClockPlayer, self).__init__(event, start_time, end_time)
        self.start_time = start_time
        self.end_time = end_time
        self.pre_roll = pre_roll
        self.post_roll = post_roll        
      
    def init(self):

        # we handle shutdown for this process
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        # make sure this node doesn't use sim time
        rospy.set_param('use_sim_time', False)

        rospy.init_node('mongodb_playback_clock_player')
    
        # switch to simulated time, note that as this is after the init_node, this node DOES NOT use sim time
        rospy.set_param('use_sim_time', True)

        # notify everyone else that they can move on
        self.event.set()


    def run(self, running):

        self.init()

        start = self.start_time - self.pre_roll
        end = self.end_time + self.post_roll
        
        # topic to public clock on
        clock_pub = rospy.Publisher('/clock', Clock)

        # start value
        clock_msg = Clock(clock=start)

        # timing details, should be moved to constructor parameters
        updates_hz = 10000.0
        rate = rospy.Rate(updates_hz)
        # this assumes close to real-time playback
        update = rospy.Duration(1.0 / updates_hz)

        while running.value and clock_msg.clock <= end:
            # publish time
            clock_pub.publish(clock_msg)

            # update time
            clock_msg.clock += update

            rate.sleep()

        rospy.loginfo('All done here')
        running.value = False


 


class MongoPlayback(object):
    """ Plays back stored topics from the mongodb_store """

    def __init__(self): 
        super(MongoPlayback, self).__init__() 

        self.mongodb_host = rospy.get_param("mongodb_host")
        self.mongodb_port = rospy.get_param("mongodb_port")
        self.mongo_client=MongoClient(self.mongodb_host, self.mongodb_port)

        
    def setup(self, database_name, req_topics):
        """ Read in details of requested playback collections. """

        database = self.mongo_client[database_name] 
        collection_names = database.collection_names(include_system_collections=False) 

        if len(req_topics) > 0:             
            topics = req_topics.intersection(collection_names) 
            dropped = req_topics.difference(topics)
            if(len(dropped) > 0):
                rospy.logwarn('Dropped non-existant requested topics for playback: %s' % dropped)
        else: 
            topics = set(collection_names)

        rospy.loginfo('Playing back topics %s' % topics)

        # create mongo collections
        collections = [database[collection_name] for collection_name in topics]

        # make sure they're easily accessible by time
        for collection in collections:
            collection.ensure_index(TIME_KEY)


        # get the min and max time across all collections, conver to ros time
        start_time = to_ros_time(min(map(min_time, collections)))
        end_time =  to_ros_time(max(map(max_time, collections)))

        # we don't need a connection any more
        self.mongo_client.disconnect()

        # rospy.loginfo('Playing back from %s' % to_datetime(start_time))
        # rospy.loginfo('.............. to %s' % to_datetime(end_time))

        self.event = multiprocessing.Event()

        # create clock thread        
        pre_roll = rospy.Duration(10)
        post_roll = rospy.Duration(10)
        self.clock_player = ClockPlayer(self.event, start_time, end_time, pre_roll, post_roll)

        # create playback objects
        self.players = map(lambda c: TopicPlayer(self.mongodb_host, self.mongodb_port, database_name, c, self.event, start_time - pre_roll, end_time + post_roll), topics)       


    def start(self):
        self.clock_player.start()

        self.event.wait()
        self.event.clear()

        for player in self.players:
            player.start()


    def join(self):

        self.clock_player.join()
        for player in self.players:
            player.join()



    def stop(self):
        print 'Shutdown requested'
        self.clock_player.stop()
        for player in self.players:
            player.stop()


    def is_running(self):
        return self.clock_player.is_running()


def main(argv):

    playback = MongoPlayback()

    def signal_handler(signal, frame):
        playback.stop()

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)


    myargv = rospy.myargv(argv=argv)
    topics = myargv[1:] 

    database_name = 'roslog'
    playback.setup(database_name, set(topics))
    playback.start()
    
    playback.join()
    


# processes load main so move init_node out 
if __name__ == "__main__":
    main(sys.argv)
