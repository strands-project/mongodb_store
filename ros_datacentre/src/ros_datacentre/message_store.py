import rospy
import ros_datacentre_msgs.srv as dc_srv
from ros_datacentre_msgs.msg import StringPair 
import StringIO
import copy

class MessageStoreProxy:
	def __init__(self, service='/message_store/insert', database='not', collection='yet'):
		self.service = service
		self.database = database
		self.collection = collection
		rospy.wait_for_service(service)
		self.insertSrv = rospy.ServiceProxy(service, dc_srv.MongoInsertMsg)

	def insertNamed(self, name, message, meta = {}):
		meta_copy = copy.copy(meta)
		meta_copy["name"] = name
		self.insert(message, meta_copy)
	

	def insert(self, message, meta = {}):
		# assume meta is a dict, convert k/v to tuple pairs 
		meta_tuple = tuple(StringPair(v, k) for k, v in meta.iteritems())
		print message._type 
		buf=StringIO.StringIO() 
		message.serialize(buf) 
		serialised_msg=buf.getvalue() 
		self.insertSrv(self.database, self.collection, message._type, serialised_msg, meta_tuple)
