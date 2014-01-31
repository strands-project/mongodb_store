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

    def type_to_class_string(self, type):
        """ 
        Takes a ROS msg type and turns it into a Python module and class name. 
        E.g. from 
        geometry_msgs/Pose 
        to
        geometry_msgs.msg._Pose.Pose
        """
        print type
        parts = type.split('/')
        cls_string = "%s.msg._%s.%s" % (parts[0], parts[1], parts[1])
        return cls_string

    def insert_ros_srv(self, req):
        """
        Receives a 
        """
        print req.type
        # get class from type
        # todo: cache classes (if this is an overhead)
        cls_string = self.type_to_class_string(req.type)
        cls = self.load_class(cls_string)
        # instantiate an object from the class
        obj = cls()
        # deserialize data into object
        obj.deserialize(req.msg)
        # convert input tuple to dict
        meta = dict((pair.second, pair.first) for pair in req.meta)
        # get requested collection from the db, creating if necessary
        collection = self._mongo_client[req.database][req.collection]
        return str(dc_util.store_message(collection, obj, meta))
        
    insert_ros_srv.type=dc_srv.MongoInsertMsg
                                              

if __name__ == '__main__':
    store = MessageStore()
    
    rospy.spin()
