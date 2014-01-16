#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "ros_datacentre_msgs/MongoInsertMsg.h"

#include <sstream>

using namespace geometry_msgs;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_ros_datacentre_cpp_client");
  ros::NodeHandle nh;


  Pose p;

  ros_datacentre_msgs::MongoInsertMsg msg;
  msg.request.database = "not";
  msg.request.collection = "yet";
  msg.request.type = ros::message_traits::DataType<Pose>::value();
  
  ros::ServiceClient client = nh.serviceClient<ros_datacentre_msgs::MongoInsertMsg>("/message_store/insert");


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

  return 0;
}


