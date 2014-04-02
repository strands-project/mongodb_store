#!/usr/bin/env python

import rospy
import subprocess
import sys
import os
import re
import signal
import errno
from std_srvs.srv import *

import ros_datacentre.util

if not ros_datacentre.util.check_for_pymongo():
    sys.exit(1)
    
import pymongo

class MongoServer(object):
    def __init__(self):
        rospy.init_node("mongodb_server", anonymous=True)#, disable_signals=True)
        rospy.on_shutdown(self._on_node_shutdown)

        # Get the database path
        self._db_path = rospy.get_param("~database_path", "/opt/ros/ros_datacentre")

        # What server does mongodb reside
        self._mongo_host = rospy.get_param("datacentre_host", "localhost")
        rospy.set_param("datacentre_host",self._mongo_host)
        self._mongo_port = rospy.get_param("datacentre_port", 27017)
        rospy.set_param("datacentre_port",self._mongo_port)
        rospy.loginfo("Mongo server address: "+self._mongo_host+":"+str(self._mongo_port))
    

        # Check that mongodb is installed
        try:
            mongov = subprocess.check_output(["mongod","--version"])
            match = re.search("db version v(\d+\.\d+\.\d+)",mongov)
            self._mongo_version=match.group(1)
        except subprocess.CalledProcessError:
            rospy.logerr("Can't find MongoDB executable. Is it installed?\nInstall it with  \"sudo apt-get install mongodb\"")
            sys.exit(1)
        rospy.loginfo("Found MongoDB version " + self._mongo_version)

        # Check that the provided db path exists.
        if not os.path.exists(self._db_path):
            rospy.logerr("Can't find database at supplied path " + self._db_path + ". If this is a new DB, create it as an empty directory.")
            sys.exit(1)

        # Advertise ros services for db interaction
        self._shutdown_srv = rospy.Service("/datacentre/shutdown", Empty, self._shutdown_srv_cb)
        self._wait_ready_srv = rospy.Service("/datacentre/wait_ready",Empty,self._wait_ready_srv_cb)

        # Has the db already gone down, before the ros node?
        self._gone_down = False

        self._ready = False # is the db ready: when mongo says "waiting for connection"
        
        # Start the mongodb server
        self._mongo_loop()

        
        
    def _mongo_loop(self):

        # Blocker to prevent Ctrl-C being passed to the mongo server
        def block_mongo_kill():
            os.setpgrp()
#            signal.signal(signal.SIGINT, signal.SIG_IGN)
        
        self._mongo_process = subprocess.Popen(["mongod","--dbpath",self._db_path,"--port",str(self._mongo_port)],
                                         stdout=subprocess.PIPE,
                                         preexec_fn = block_mongo_kill)

        while self._mongo_process.poll() is None:# and not rospy.is_shutdown():
            try:
                stdout = self._mongo_process.stdout.readline()
            except IOError, e: # probably interupt because shutdown cut it up
                if e.errno == errno.EINTR:
                    continue
                else:
                    raise
            if stdout is not None:
                if stdout.find("ERROR") !=-1:
                    rospy.logerr(stdout.strip())
                else:
                    rospy.loginfo(stdout.strip())

                if stdout.find("waiting for connections on port") !=-1:
                    self._ready=True

        if not rospy.is_shutdown():
            rospy.logerr("MongoDB process stopped!")
            
        if self._mongo_process.returncode!=0:
            rospy.logerr("Mongo process error! Exit code="+str(self._mongo_process.returncode))

        self._gone_down = True
        self._ready=False

    def _on_node_shutdown(self):
        rospy.loginfo("Shutting down datacentre")
        if self._gone_down:
            rospy.logwarn("It looks like Mongo already died. Watch out as the DB might need recovery time at next run.")
            return
        try:
            c = pymongo.MongoClient(self._mongo_host,self._mongo_port)
        except ConnectionError, c:
            pass
        try:
            c.admin.command("shutdown")
        except pymongo.errors.AutoReconnect, a:
            pass


    def _shutdown_srv_cb(self,req):
        rospy.signal_shutdown("Shutdown request..")
        return EmptyResponse()

    def _wait_ready_srv_cb(self,req):
        while not self._ready:
            rospy.sleep(0.1)
        return EmptyResponse()
            

if __name__ == '__main__':
    server = MongoServer()

