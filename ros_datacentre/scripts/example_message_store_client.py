#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import ros_datacentre_msgs.srv as dc_srv
import ros_datacentre.util as dc_util
from geometry_msgs.msg import Pose, Point, Quaternion
import StringIO


if __name__ == '__main__':
    rospy.init_node("example_message_store_client")
    rospy.wait_for_service('/message_store/insert')

    p = Pose(Point(0, 1, 2), Quaternion(3, 4,  5, 6))

    try:
        insert = rospy.ServiceProxy('/message_store/insert', dc_srv.MongoInsertMsg)
        database="not"
        collection="yet"                
        print p._type
        buf=StringIO.StringIO()
        p.serialize(buf)
        msg=buf.getvalue()
        insert(database, collection, p._type, msg)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        
