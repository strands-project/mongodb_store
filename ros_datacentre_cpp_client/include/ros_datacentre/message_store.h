#include "ros/ros.h"
#include "ros_datacentre_msgs/MongoInsertMsg.h"

namespace ros_datacentre {

class MessageStoreProxy
{
public:

	/**

	**/
	MessageStoreProxy(ros::NodeHandle handle, 
		const std::string & _service = "/message_store/insert", 
		const std::string & _database = "not", 
		const std::string & _collection = "yet") {
		init(handle, _service, _database, _collection);
	}


	MessageStoreProxy(const MessageStoreProxy& _rhs) :
		m_database(_rhs.m_database),
		m_collection(_rhs.m_collection),
		m_client(_rhs.m_client)
	{}


	~MessageStoreProxy() {}

	template<typename MsgType> 
	void insert(const MsgType & _msg) {
		insert(_msg, m_database, m_collection);
	}

	template<typename MsgType> 
	void insert(const MsgType & _msg, 
		const std::string & _database, 
		const std::string & _collection) {
		
  		//Create message with basic fields
  		ros_datacentre_msgs::MongoInsertMsg msg;
  		msg.request.database = _database;
  		msg.request.collection = _collection;
  		msg.request.type = ros::message_traits::DataType<MsgType>::value();
 
	 	//how long the data will be
  		uint32_t serial_size = ros::serialization::serializationLength(_msg);
 	 	//set msg vector to this size
  		msg.request.msg.resize(serial_size);
  		//serialise the object into the vector via this stream
   		ros::serialization::OStream stream(&(msg.request.msg[0]), serial_size);
  		ros::serialization::serialize(stream, _msg);

  		//sent data over
  		m_client.call(msg);
	}


	private:

	void init(ros::NodeHandle handle, const std::string & _service, 
		const std::string & _database, const std::string & _collection) {		
		m_client = handle.serviceClient<ros_datacentre_msgs::MongoInsertMsg>(_service);
		m_database = _database;
		m_collection = _collection;
	}

protected:
	std::string m_database;
	std::string m_collection;
	ros::ServiceClient m_client;
};








}