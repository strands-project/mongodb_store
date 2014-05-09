#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import rospy
import actionlib
import pymongo
import os
import shutil
import subprocess
from ros_datacentre_msgs.msg import  MoveEntriesAction, MoveEntriesFeedback
from datetime import *

class Replicator(object):
    def __init__(self):

        # this is just a test, connections are remade every call for long-running processes
        master, extras = self.make_connections()
        if master is None:
            raise Exception("No master datacentre found using datacentre_host and datacentre_port")

        self.server = actionlib.SimpleActionServer('move_datacentre_entries', MoveEntriesAction, self.move_entries, False)
        self.server.start()
        self.dump_path = '/tmp/mongodb_replicator'

        self.make_path()
        self.remove_path()


    def make_path(self):        
        if not os.path.isdir(self.dump_path):
            os.makedirs(self.dump_path) 
        elif not os.access(self.dump_path, os.W_OK):
            raise Exception('Cannot write to dump path: %s' % self.dump_path)
        
    def remove_path(self):
        shutil.rmtree(self.dump_path)

    def make_connections(self):
        datacentre_host = rospy.get_param("datacentre_host")
        datacentre_port = rospy.get_param("datacentre_port") 
        master = None
        try:
            master = pymongo.MongoClient(datacentre_host, datacentre_port)
        except pymongo.errors.ConnectionFailure, e:
            rospy.logwarn('Could not connect to master datacentre at %s:%s' % (datacentre_host, datacentre_port))
            return None, None


        extras = rospy.get_param('ros_datacentre_extras', [])
        extra_clients = []
        for extra in extras:
            try:
                extra_clients.append(pymongo.MongoClient(extra[0], extra[1]))
            except pymongo.errors.ConnectionFailure, e:
                rospy.logwarn('Could not connect to extra datacentre at %s:%s' % (extra[0], extra[1]))


        rospy.loginfo('Replicating content from %s:%s to a futher %s datacentres', datacentre_host, datacentre_port, len(extra_clients))

        return master, extra_clients

    def move_entries(self, goal):

        # create place to put temp stuf
        self.make_path()
        
        # don't use the connections, just sanity check their existence
        master, extras = self.make_connections()
        

        completed = []
        feedback = MoveEntriesFeedback(completed=completed)

        for collection in goal.collections:                    
            self.do_dump(collection, master)

        self.do_restore(extras)



        # clean up
        # self.remove_path()

        self.server.set_succeeded()    

    def do_restore(self, extras, db='message_store'):       
        # restore collection to extras
        for extra in extras:            
            rest_args = ['mongorestore',  '--host',  extra.host, '--port',  str(extra.port), self.dump_path]
            subprocess.call(rest_args)    


    def do_dump(self, collection, master, db='message_store'):       
        # dump collection
        args = ['mongodump',  '--host',  master.host, '--port',  str(master.port), '--db', db, '--collection', collection, '-o', self.dump_path]
        subprocess.call(args)



if __name__ == '__main__':
    rospy.init_node("mongodb_replicator")

    store = Replicator()
    
    rospy.spin()
