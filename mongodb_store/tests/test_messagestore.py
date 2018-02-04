#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import unittest
import random
from mongodb_store.message_store import MessageStoreProxy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

class TestMessageStoreProxy(unittest.TestCase):

    def test_add_message(self):
        msg_store = MessageStoreProxy()
        POSE_NAME = "__test__pose__"
        p = Pose(Point(0, 1, 2), Quaternion(0, 0, 0, 1))

        
        # insert a pose object with a name
        msg_store.insert_named(POSE_NAME, p)
 
        # get it back with a name
        stored, meta = msg_store.query_named(POSE_NAME, Pose._type)

        self.assertIsInstance(stored, Pose)
        self.assertEqual(stored.position.x, p.position.x)
        self.assertEqual(stored.position.y, p.position.y)
        self.assertEqual(stored.position.z, p.position.z)

        self.assertEqual(stored.orientation.x, p.orientation.x)
        self.assertEqual(stored.orientation.y, p.orientation.y)
        self.assertEqual(stored.orientation.z, p.orientation.z)
        self.assertEqual(stored.orientation.w, p.orientation.w)

        p.position.x = 666

        msg_store.update_named(POSE_NAME, p)

        # get it back with a name
        updated = msg_store.query_named(POSE_NAME, Pose._type)[0]

        self.assertEqual(updated.position.x, p.position.x)

        # # try to get it back with an incorrect name
        wrong_name = "thid name does not exist in the datacentre"
        none_item = msg_store.query_named(wrong_name, Pose._type)[0]
        self.assertIsNone(none_item)

        # # get all non-existant typed objects, so get an empty list back
        none_query = msg_store.query( "not my type")
        self.assertEqual(len(none_query), 0)

        # add 100 query and sort by date inserted.
        for i in range(100):
            p = Pose(Point(0, 0, 0), Quaternion(i, 0, 100, 1))
            msg_store.insert(p)
            
        result = msg_store.query(Pose._type, message_query={ 'orientation.z': {'$gt': 10} }, sort_query=[("$natural", -1)])
        self.assertEqual(len(result), 100)
        self.assertEqual(result[0][0].orientation.x, 99)

        # get documents with limit
        result_limited = msg_store.query(Pose._type, message_query={'orientation.z': {'$gt': 10} }, sort_query=[("$natural", 1)], limit=10)
        self.assertEqual(len(result_limited), 10)
        self.assertListEqual([int(doc[0].orientation.x) for doc in result_limited], range(10))

	#get documents without "orientation" field
	result_no_id = msg_store.query(Pose._type, message_query={}, projection_query={"orientation": 0})
        for doc in result_no_id:
		self.assertEqual(int(doc[0].orientation.z),0 ) 
        
        
	
        
        # must remove the item or unittest only really valid once
        print meta["_id"]
        print str(meta["_id"])
        deleted = msg_store.delete(str(meta["_id"]))
        self.assertTrue(deleted)

    def test_add_message_no_wait(self):
        msg_store = MessageStoreProxy()
        count_before_insert = len(msg_store.query(Pose._type, meta_query={ "no_wait": True }))
        p = Pose(Point(0, 1, 2), Quaternion(0, 0, 0, 1))
        for i in range(10):
            msg_store.insert(p, meta={"no_wait": True }, wait=False)
        rospy.sleep(2)
        count_after_insert = len(msg_store.query(Pose._type, meta_query={ "no_wait": True }))
        self.assertTrue(count_after_insert > count_before_insert)

    def test_non_ascii(self):
        msg_store = MessageStoreProxy()
        msg = String(data="こんにちは")  # non ascii string
        doc_id = msg_store.insert(msg)

        try:
            qmsg, _ = msg_store.query_id(doc_id, String._type)
            self.assertEqual(msg.data, qmsg.data)
        except rospy.service.ServiceException:
            self.fail("non ascii unicode string cannot be queried")


if __name__ == '__main__':
    import rostest
    PKG = 'mongodb_store'
    rospy.init_node('test_message_store_proxy')
    rostest.rosrun(PKG, 'test_message_store_proxy', TestMessageStoreProxy)
    
 
