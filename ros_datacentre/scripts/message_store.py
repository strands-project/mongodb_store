#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import rospy
import ros_datacentre_msgs.srv as dc_srv
import ros_datacentre.util as dc_util
import pymongo
import importlib
# from std_srvs.srv import *

# For testing
from geometry_msgs.msg import Pose, Point, Quaternion




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

    def load_class(self, full_class_string):
        """
        dynamically load a class from a string
        shamelessly ripped from: http://thomassileo.com/blog/2012/12/21/dynamically-load-python-modules-or-classes/
        """
        class_data = full_class_string.split(".")
        module_path = ".".join(class_data[:-1])
        class_str = class_data[-1]
        module = importlib.import_module(module_path)
        # Finally, we retrieve the Class
        return getattr(module, class_str)

                                               
    # def insert_ros_srv(self, req):
    #     print req.type
    #     # get class from type
    #     # todo: cache classes (if this is an overhead)
    #     cls = self.load_class(req.type)
    #     # instantiate an object from the class
    #     obj = cls()
    #     # deserialize data into object
    #     obj.deserialize(req.msg)
    #     print obj
    #     collection = self._mongo_client[req.database][req.collection]
    def insert_ros_srv(self, req):
        print req
        obj = Pose()
        # obj = cls()
        # deserialize data into object
        obj.deserialize(str(req.msg))
        print obj
        
    insert_ros_srv.type=dc_srv.MongoInsertMsg
                                              

if __name__ == '__main__':
    store = MessageStore()
    
    rospy.spin()
