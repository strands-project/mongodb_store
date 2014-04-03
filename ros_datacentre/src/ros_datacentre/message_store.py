import rospy
import ros_datacentre_msgs.srv as dc_srv
import ros_datacentre.util as dc_util
from ros_datacentre_msgs.msg import StringPair, StringPairList, SerialisedMessage
from bson import json_util
from bson.objectid import ObjectId
import copy


class MessageStoreProxy:
	def __init__(self, service_prefix='/message_store', database='message_store', collection='message_store'):
		self.database = database
		self.collection = collection
		insert_service = service_prefix + '/insert'
		update_service = service_prefix + '/update'
		query_ids_service = service_prefix + '/query_messages'
		rospy.logdebug("Waiting for services...")
		rospy.wait_for_service(insert_service)
		rospy.wait_for_service(update_service)
		rospy.wait_for_service(query_ids_service)
		rospy.logdebug("Done")
		self.insert_srv = rospy.ServiceProxy(insert_service, dc_srv.MongoInsertMsg)
		self.update_srv = rospy.ServiceProxy(update_service, dc_srv.MongoUpdateMsg)
		self.query_id_srv = rospy.ServiceProxy(query_ids_service, dc_srv.MongoQueryMsg)


	def insert_named(self, name, message, meta = {}):
		# create a copy as we're modifying it
		meta_copy = copy.copy(meta)
		meta_copy["name"] = name
		return self.insert(message, meta_copy)
	

	def insert(self, message, meta = {}):
		# assume meta is a dict, convert k/v to tuple pairs 
		meta_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json_util.dumps(meta)),)
		serialised_msg = dc_util.serialise_message(message)
		return self.insert_srv(self.database, self.collection, serialised_msg, StringPairList(meta_tuple))

	def query_id(self, id, type):
		return self.query(type, {'_id': ObjectId(id)}, {}, True)

	def query_named(self, name, type, single = True, meta = {}):
		# create a copy as we're modifying it
		meta_copy = copy.copy(meta)
		meta_copy["name"] = name
		return self.query(type, {}, meta_copy, single)

	def update_named(self, name, message, meta = {}, upsert = False):
		meta_query = {}
		meta_query["name"] = name

		# make sure the name goes into the meta info after update
		meta_copy = copy.copy(meta)
		meta_copy["name"] = name

		return self.update(message, meta_copy, {}, meta_query, upsert)		

	def update(self, message, meta = {}, message_query = {}, meta_query = {},  upsert = False):
		# serialise the json queries to strings using json_util.dumps
		message_query_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json_util.dumps(message_query)),)
		meta_query_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json_util.dumps(meta_query)),)
		meta_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json_util.dumps(meta)),)
		return self.update_srv(self.database, self.collection, upsert, StringPairList(message_query_tuple), StringPairList(meta_query_tuple), dc_util.serialise_message(message), StringPairList(meta_tuple))		


	"""
	Returns [message, meta] where message is the queried message and meta a dictionary of meta information. If single is false returns a list of these lists.
	"""
	def query(self, type, message_query = {}, meta_query = {}, single = False):
		# assume meta is a dict, convert k/v to tuple pairs for ROS msg type

		# serialise the json queries to strings using json_util.dumps
		message_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json_util.dumps(message_query)),)
		meta_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json_util.dumps(meta_query)),)

		# a tuple of SerialisedMessages
		response = self.query_id_srv(self.database, self.collection, type, single, StringPairList(message_tuple), StringPairList(meta_tuple))		

		# print response

		if response.messages is None:
			messages = []
			metas = []
		else:
			messages = map(dc_util.deserialise_message, response.messages) 
			metas = map(dc_util.string_pair_list_to_dictionary, response.metas)

		if single:
			if len(messages) > 0:
				return [messages[0], metas[0]]
			else:				
				return [None, None]
		else:
			return zip(messages,metas)
