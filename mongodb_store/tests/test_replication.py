#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
from bson import json_util
import pymongo
import rospy
import subprocess
import unittest
from mongodb_store.util import import_MongoClient, wait_for_mongo
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Wrench, Pose


def get_script_path():
    test_dir = os.path.dirname(os.path.realpath(__file__))
    pkg_dir = os.path.dirname(test_dir)
    return os.path.join(pkg_dir, "scripts", "replicator_client.py")


class TestReplication(unittest.TestCase):
    def test_replication(self):
        replication_db = "replication_test"
        replication_col = "replication_test"
        # connect to destination for replication
        try:
            self.assertTrue(wait_for_mongo(ns="/datacentre2"), "wait for mongodb server")
            dst_client = import_MongoClient()("localhost", 49163)
            count = dst_client[replication_db][replication_col].count()
            self.assertEqual(count, 0, "No entry in destination")
        except pymongo.errors.ConnectionFailure:
            self.fail("Failed to connect to destination for replication")

        # insert an entry to move
        self.assertTrue(wait_for_mongo(), "wait for mongodb server")
        msg_store = MessageStoreProxy(
            database=replication_db, collection=replication_col)
        msg = Wrench()
        msg_name = "replication test message"
        self.assertIsNotNone(msg_store.insert_named(msg_name, msg), "inserted message")

        # move entries
        rospy.sleep(3)
        retcode = subprocess.check_call([
            get_script_path(),
            '--move-before', '0',
            replication_db, replication_col])
        self.assertEqual(retcode, 0, "replicator_client returns code 0")

        # check if replication was succeeded
        rospy.sleep(3)
        count = dst_client[replication_db][replication_col].count()
        self.assertGreater(count, 0, "entry moved to the destination")

        # test deletion after move
        data, meta = msg_store.query_named(msg_name, Wrench._type)
        self.assertIsNotNone(data, "entry is still in source")
        retcode = subprocess.check_call([
            get_script_path(),
            '--move-before', '0',
            '--delete-after-move',
            replication_db, replication_col])
        self.assertEqual(retcode, 0, "replicator_client returns code 0")
        data, meta = msg_store.query_named("replication test", Wrench._type)
        self.assertIsNone(data, "moved entry is deleted from source")

    def test_replication_with_query(self):
        replication_db = "replication_test_with_query"
        replication_col = "replication_test_with_query"
        # connect to destination for replication
        try:
            self.assertTrue(wait_for_mongo(ns="/datacentre2"), "wait for mongodb server")
            dst_client = import_MongoClient()("localhost", 49163)
            count = dst_client[replication_db][replication_col].count()
            self.assertEqual(count, 0, "No entry in destination")
        except pymongo.errors.ConnectionFailure:
            self.fail("Failed to connect to destination for replication")

        # insert an entry to move
        self.assertTrue(wait_for_mongo(), "wait for mongodb server")
        msg_store = MessageStoreProxy(
            database=replication_db, collection=replication_col)
        for i in range(5):
            msg = Wrench()
            msg.force.x = i
            msg_store.insert(msg)
            msg = Pose()
            msg.position.x = i
            msg_store.insert(msg)

        # move entries with query
        rospy.sleep(3)
        query = {'_meta.stored_type': Pose._type}
        retcode = subprocess.check_call([
            get_script_path(),
            '--move-before', '0',
            '--query', json_util.dumps(query),
            replication_db, replication_col])
        self.assertEqual(retcode, 0, "replicator_client returns code 0")

        # check if replication was succeeded
        rospy.sleep(3)
        count = dst_client[replication_db][replication_col].count()
        self.assertEqual(count, 5, "replicated entry exists in destination")


if __name__ == '__main__':
    import rostest
    rospy.init_node("test_replication")
    rostest.rosrun("mongodb_store", "test_replication", TestReplication)
