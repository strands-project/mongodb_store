import rospy
import ros_datacentre_msgs.srv as dc_srv
import StringIO

class MessageStoreProxy:
	def __init__(self, service, database, collection):
		self.service = service
		self.database = database
		self.collection = collection
		rospy.wait_for_service(service)
		self.insertSrv = rospy.ServiceProxy(service, dc_srv.MongoInsertMsg)


	def insert(self, message):
		print message._type 
		buf=StringIO.StringIO() 
		message.serialize(buf) 
		serialised_msg=buf.getvalue() 
		self.insertSrv(self.database, self.collection, message._type, serialised_msg)
