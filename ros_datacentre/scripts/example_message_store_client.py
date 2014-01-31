#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import ros_datacentre_msgs.srv as dc_srv
import ros_datacentre.util as dc_util
from ros_datacentre.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
import StringIO


if __name__ == '__main__':
    rospy.init_node("example_message_store_client")


    msg_store = MessageStoreProxy()

    p = Pose(Point(0, 1, 2), Quaternion(3, 4,  5, 6))
  
    try:
        # insert a pose object with a name
        msg_store.insert_named("my favourite pose", p)
 
        # get it back with a name
        print msg_store.query_named("my favourite pose", Pose._type)
        # try to get it back with an incorrect name, so get None instead
        print msg_store.query_named("my favourite position", Pose._type)

        # get all poses  
        print msg_store.query({} , Pose._type)
        # get all non-existant typed objects, so get an empty list back
        print msg_store.query({} , "not my type")
        
        # get all poses where the y position is 1
        print msg_store.query({"position.y": 1} , Pose._type)

        # get all poses where the y position greater than 0
        print msg_store.query({"position.y": {"$gt": 0}} , Pose._type)

        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        
