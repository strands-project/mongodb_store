import rospy
import ros_datacentre_msgs.srv as dc_srv
from ros_datacentre_msgs.msg import StringPair 
import StringIO
import copy

class MessageStoreProxy:
	def __init__(self, service_prefix='/message_store', database='not', collection='yet'):
		self.database = database
		self.collection = collection
		insert_service = service_prefix + '/insert'
		query_ids_service = service_prefix + '/query_ids'
		rospy.wait_for_service(insert_service)
		rospy.wait_for_service(query_ids_service)
		self.insertSrv = rospy.ServiceProxy(insert_service, dc_srv.MongoInsertMsg)
		self.queryIdSrv = rospy.ServiceProxy(query_ids_service, dc_srv.MongoQueryMsg)

	def insert_named(self, name, message, meta = {}):
		# create a copy as we're modifying it
		meta_copy = copy.copy(meta)
		meta_copy["name"] = name
		self.insert(message, meta_copy)
	

	def insert(self, message, meta = {}):
		# assume meta is a dict, convert k/v to tuple pairs 
		meta_tuple = tuple(StringPair(k, v) for k, v in meta.iteritems())
		print message._type 
		buf=StringIO.StringIO() 
		message.serialize(buf) 
		serialised_msg=buf.getvalue() 
		self.insertSrv(self.database, self.collection, message._type, serialised_msg, meta_tuple)

	def query_named(self, name, message, meta = {}):
		# create a copy as we're modifying it
		meta_copy = copy.copy(meta)
		meta_copy["name"] = name
		self.query({}, message, meta_copy, True)

	def query(self, message_query, message, meta_query = {}, single = False):
		# assume meta is a dict, convert k/v to tuple pairs 
		message_tuple = tuple(StringPair(k, v) for k, v in message_query.iteritems())
		meta_tuple = tuple(StringPair(k, v) for k, v in meta_query.iteritems())
				
		ids = self.queryIdSrv(self.database, self.collection, message._type, single, message_tuple, meta_tuple)
		print ids