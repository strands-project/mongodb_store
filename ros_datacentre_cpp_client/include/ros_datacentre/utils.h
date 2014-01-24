#include "ros/ros.h"
#include "ros_datacentre_msgs/MongoInsertMsg.h"

namespace ros_datacentre {

/*
  Pose p;

  ros_datacentre_msgs::MongoInsertMsg msg;
  msg.request.database = "not";
  msg.request.collection = "yet";
  msg.request.type = ros::message_traits::DataType<Pose>::value();
  
  ros::ServiceClient client = nh.serviceClient<ros_datacentre_msgs::MongoInsertMsg>();


   // std::stringstream ss;
   //  ss << ros::message_traits::DataType<Pose>::value() << " " << p;
   //  ROS_INFO("%s", ss.str().c_str());


  //how long the data will be
  uint32_t serial_size = ros::serialization::serializationLength(p);
  //set msg vector to this size
  msg.request.msg.resize(serial_size);
  //serialise the object into the vector via this stream
   ros::serialization::OStream stream(&(msg.request.msg[0]), serial_size);
  ros::serialization::serialize(stream, p);

  //sent data over
  client.call(msg);
*/

class MessageStoreProxy
{
public:


	/**

	**/
	MessageStoreProxy(ros::NodeHandle handle, std::string _service, std::string _database, std::string _collection) {
		init(handle, _service, _database, _collection);
	}

	/**
	* Default contructor with sensible default values.
	**/
	MessageStoreProxy(ros::NodeHandle handle)  {
		init(handle, "/message_store/insert",  "not", "yet");
	}

	~MessageStoreProxy() {}

	private:

	void init(ros::NodeHandle handle, std::string _service, std::string _database, std::string _collection) {		
		m_client = handle.serviceClient<ros_datacentre_msgs::MongoInsertMsg>("/message_store/insert");
	}

protected:
	std::string m_database;
	std::string m_collection;
	ros::ServiceClient m_client;
};








}