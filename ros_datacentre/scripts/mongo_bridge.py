#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Bridges mongo to ros, providing services to query the database.
Not efficient, so should be only used when no other option is available; if
you can use a mongodb client library like pymongo, use it, if you need to query
from client side javascript on a webpage with rosbridge/librosjs then use this.

"""

import rospy
import ros_datacentre.util as dc_util
from ros_datacentre.srv import *
import pymongo
import bson.json_util 

class MongoBridge(object):
    def __init__(self):
        rospy.init_node("mongo_bridge")

        have_dc = dc_util.wait_for_mongo()
        if not have_dc:
            raise Exception("No Datacentre?")

        self._mongo_client=pymongo.Connection(rospy.get_param("datacentre_host"),
                                              rospy.get_param("datacentre_port") )

        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service=getattr(self, attr)
                rospy.Service("/mongo_bridge/"+attr[:-8], service.type, service)


    def find_ros_srv(self, req):
        collection = self._mongo_client[req.db][req.collection]
        res=collection.find(bson.json_util.loads(req.query))
        docs=[i for i in res]
        return bson.json_util.dumps(docs)
    find_ros_srv.type=MongoFind

    def update_ros_srv(self,req):
        collection = self._mongo_client[req.db][req.collection]
        res=collection.update(bson.json_util.loads(req.query),
                              bson.json_util.loads(req.update))
        return bson.json_util.dumps(docs)
    update_ros_srv.type=MongoUpdate
                                               
    def insert_ros_srv(self,req):
        collection = self._mongo_client[req.db][req.collection]
        res=collection.insert(bson.json_util.loads(req.document))
        return bson.json_util.dumps(res)
    insert_ros_srv.type=MongoInsert
                                              

if __name__ == '__main__':
    bridge = MongoBridge()
    
    rospy.spin()
