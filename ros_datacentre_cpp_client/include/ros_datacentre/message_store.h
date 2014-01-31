#include "ros/ros.h"
#include "ros_datacentre_msgs/MongoInsertMsg.h"
#include "ros_datacentre_msgs/StringPair.h"

namespace ros_datacentre {

ros_datacentre_msgs::StringPair makePair(const std::string & _first, const std::string & _second) {
	ros_datacentre_msgs::StringPair pair;
	pair.first = _first;
	pair.second = _second;
	return pair;
}


class MessageStoreProxy
{
public:

	/**

	**/
	MessageStoreProxy(ros::NodeHandle handle, 
		const std::string & _service = "/message_store/insert", 
		const std::string & _database = "not", 
		const std::string & _collection = "yet") :
		m_client(handle.serviceClient<ros_datacentre_msgs::MongoInsertMsg>(_service)),
		m_database(_database),
		m_collection(_collection)
	{}

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
	void insertNamed(const std::string & _name, const MsgType & _msg, 
		const std::vector<ros_datacentre_msgs::StringPair> & _meta = EMPTY_META) {
		
		//make a copy so we can add stuff
		std::vector<ros_datacentre_msgs::StringPair> meta = _meta;
		meta.push_back(makePair("name", _name));
		insert(_msg, m_database, m_collection, meta);
	}

	template<typename MsgType> 
	void insert(const MsgType & _msg, 
		const std::string & _database, 
		const std::string & _collection, 
		const std::vector<ros_datacentre_msgs::StringPair> & _meta = EMPTY_META) {

  		//Create message with basic fields
  		ros_datacentre_msgs::MongoInsertMsg msg;
  		msg.request.database = _database;
  		msg.request.collection = _collection;
  		msg.request.message.type = ros::message_traits::DataType<MsgType>::value();
 		
 		//if there's no meta then no copying is necessary
  		if(_meta.size() > 0) {
 			msg.request.meta = _meta;
		}

	 	//how long the data will be
  		uint32_t serial_size = ros::serialization::serializationLength(_msg);
 	 	//set msg vector to this size
  		msg.request.message.msg.resize(serial_size);
  		//serialise the object into the vector via this stream
   		ros::serialization::OStream stream(&(msg.request.message.msg[0]), serial_size);
  		ros::serialization::serialize(stream, _msg);

  		//sent data over
  		m_client.call(msg);
	}



protected:
	std::string m_database;
	std::string m_collection;
	ros::ServiceClient m_client;

	//an empty vector to save recreating one whenever meta info is not provided
	static const std::vector<ros_datacentre_msgs::StringPair> EMPTY_META;
};


const std::vector<ros_datacentre_msgs::StringPair> MessageStoreProxy::EMPTY_META =  std::vector<ros_datacentre_msgs::StringPair>();







}