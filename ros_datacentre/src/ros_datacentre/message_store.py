import rospy
import ros_datacentre_msgs.srv as dc_srv
import ros_datacentre.util as dc_util
from ros_datacentre_msgs.msg import StringPair, SerialisedMessage
import json
import copy

class MessageStoreProxy:
	def __init__(self, service_prefix='/message_store', database='not', collection='yet'):
		self.database = database
		self.collection = collection
		insert_service = service_prefix + '/insert'
		query_ids_service = service_prefix + '/query_messages'
		rospy.wait_for_service(insert_service)
		rospy.wait_for_service(query_ids_service)
		self.insert_srv = rospy.ServiceProxy(insert_service, dc_srv.MongoInsertMsg)
		self.query_id_srv = rospy.ServiceProxy(query_ids_service, dc_srv.MongoQueryMsg)

	def insert_named(self, name, message, meta = {}):
		# create a copy as we're modifying it
		meta_copy = copy.copy(meta)
		meta_copy["name"] = name
		self.insert(message, meta_copy)
	

	def insert(self, message, meta = {}):
		# assume meta is a dict, convert k/v to tuple pairs 
		meta_tuple = tuple(StringPair(k, v) for k, v in meta.iteritems())
		serialised_msg = dc_util.serialise_message(message)
		self.insert_srv(self.database, self.collection, serialised_msg, meta_tuple)

	def query_named(self, name, type, single = True, meta = {}):
		# create a copy as we're modifying it
		meta_copy = copy.copy(meta)
		meta_copy["name"] = name
		return self.query(type, {}, meta_copy, single)

	def query(self, type, message_query = {}, meta_query = {}, single = False):
		# assume meta is a dict, convert k/v to tuple pairs for ROS msg type

		# serialise the json queries to strings using json.dumps
		message_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json.dumps(message_query)),)
		meta_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json.dumps(meta_query)),)

		# a tuple of SerialisedMessages
		response = self.query_id_srv(self.database, self.collection, type, single, message_tuple, meta_tuple)		

		if response.messages is None:
			messages = []
		else:
			messages = map(dc_util.deserialise_message, response.messages) 

		if single:
			if len(messages) > 0:
				return messages[0]
			else:
				return None
		else:
			return messages