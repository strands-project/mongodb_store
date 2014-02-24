#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import rospy
import ros_datacentre_msgs.srv as dc_srv
import ros_datacentre.util as dc_util
import pymongo
import json



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



    def insert_ros_srv(self, req):
        """
        Receives a 
        """
        # deserialize data into object
        obj = dc_util.deserialise_message(req.message)        
        # convert input tuple to dict
        meta = dict((pair.first, pair.second) for pair in req.meta)
        #  also store type information
        meta["stored_class"] = obj.__module__ + "." + obj.__class__.__name__
        meta["stored_type"] = req.message.type
        # get requested collection from the db, creating if necessary
        collection = self._mongo_client[req.database][req.collection]
        obj_id = dc_util.store_message(collection, obj, meta)
        return str(obj_id)        
    insert_ros_srv.type=dc_srv.MongoInsertMsg
                          

    def query_messages_ros_srv(self, req):
        """
        Returns t
        """
        collection = self._mongo_client[req.database][req.collection]


        # build the query doc 
        
        # load serialised json

        if len(req.message_query) > 0 and req.message_query[0].first == dc_srv.MongoQueryMsgRequest.JSON_QUERY:
            obj_query = json.loads(req.message_query[0].second)
        # else use the string pairs
        else:
            obj_query = dict((pair.first, pair.second) for pair in req.message_query)
        

        # load serialised json for meta
        if len(req.meta_query) > 0 and req.meta_query[0].first == dc_srv.MongoQueryMsgRequest.JSON_QUERY:
            meta_query = json.loads(req.meta_query[0].second)
            # prefix all keys with "_meta." to make it 
            for old_key in meta_query:
                meta_query["_meta." + old_key] = meta_query.pop(old_key)
                obj_query.update(meta_query)
        # else use the string pairs
        else:
            obj_query.update(dict(("_meta." + pair.first, pair.second) for pair in req.meta_query))


        # restrict results to have the type asked for
        obj_query["_meta.stored_type"] = req.type

        # TODO start using some string constants!

        rospy.loginfo("query document: %s", obj_query) 
        

        # this is a list of entries in dict format including meta
        entries =  dc_util.query_message(collection, obj_query, req.single)

        serialised_messages = ()

        for entry in entries:
            # load the class object for this type
            # TODO this should be the same for every item in the list, so could reuse
            cls = dc_util.load_class(entry["_meta"]["stored_class"])
            # instantiate the ROS message object from the dictionary retrieved from the db
            message = dc_util.dictionary_to_message(entry, cls)            
            # the serialise this object in order to be sent in a generic form
            serialised_messages = serialised_messages + (dc_util.serialise_message(message), )
       
        return [serialised_messages]
        
    query_messages_ros_srv.type=dc_srv.MongoQueryMsg


if __name__ == '__main__':
    store = MessageStore()
    
    rospy.spin()
