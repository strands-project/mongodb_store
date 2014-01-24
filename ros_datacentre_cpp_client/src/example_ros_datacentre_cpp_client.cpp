#include "ros_datacentre/utils.h"
#include "geometry_msgs/Pose.h"


#include <sstream>

using namespace geometry_msgs;
using namespace ros_datacentre;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_ros_datacentre_cpp_client");
  ros::NodeHandle nh;

  MessageStoreProxy messageStore(nh);



  return 0;
}


