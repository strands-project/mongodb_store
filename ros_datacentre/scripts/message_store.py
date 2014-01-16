#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import rospy
import ros_datacentre_msgs.srv
import ros_datacentre.util as dc_util
import pymongo

class MessageStore(object):
    def __init__(self):
        rospy.init_node("message_store")

        have_dc = dc_util.wait_for_mongo()
        if not have_dc:
            raise Exception("No Datacentre?")

        self._mongo_client=pymongo.MongoClient(rospy.get_param("datacentre_host"),
                                               rospy.get_param("datacentre_port") )

        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service=getattr(self, attr)
                rospy.Service("/message_store/"+attr[:-8], service.type, service)



                                               
    def insert_ros_srv(self,req):
        collection = self._mongo_client[req.database][req.collection]
        
    insert_ros_srv.type=MongoInsertMsg
                                              

if __name__ == '__main__':
    store = MessageStore()
    
    rospy.spin()
