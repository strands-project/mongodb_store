#include "ros_datacentre/message_store.h"
#include "geometry_msgs/Pose.h"
#include <boost/foreach.hpp>

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

	//Create object which does the work for us.
	MessageStoreProxy messageStore(nh);

	//This is the message we want to store
	Pose p;

	//Insert it
	messageStore.insertNamed("my pose", p);

	std::vector< boost::shared_ptr<Pose> > results;

	messageStore.queryNamed<Pose>("my favourite pose", results);

	BOOST_FOREACH( boost::shared_ptr<Pose> p,  results)
	{
		ROS_INFO_STREAM("Got one: " << *p);
	}


	return 0;
}


