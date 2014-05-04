#include "ros_datacentre/message_store.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Bool.h"

#include "ros_datacentre_msgs/StringPairList.h"

#include <sstream>
#include <utility>
#include <vector>

#include <boost/foreach.hpp>

using namespace std_msgs;
using namespace geometry_msgs;
using namespace ros_datacentre;
using namespace ros_datacentre_msgs;
using namespace std;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_multi_event_log");
	ros::NodeHandle nh;


	// let's say we have a couple of things that we need to store together
    // these could be some sensor data, results of processing etc.
    Pose pose;
    Point point;
    Quaternion quaternion;
    // note that everything that is pass to the message_store must be a ros message type
    // therefore use std_msg types for standard data types like float, int, bool, string etc
    Bool result;
    result.data = true;

	// we will store our results in a separate collection
	MessageStoreProxy messageStore(nh, "pose_process_results");

	vector< pair<string, string> > stored;
	// now add objects and store ids with the addition of type strings for safety. The types are not necessary unless you want to do some kind of reflection on this data later.
	stored.push_back( make_pair(get_ros_type(pose), messageStore.insert(pose)) );
	stored.push_back( make_pair(get_ros_type(point), messageStore.insert(point)) );
	stored.push_back( make_pair(get_ros_type(quaternion), messageStore.insert(quaternion)) );
	stored.push_back( make_pair(get_ros_type(result), messageStore.insert(result)) );

	// add these things to a single structure for storing in the db
	StringPairList spl;
	for(auto & pair : stored) {
		spl.pairs.push_back(ros_datacentre::makePair(pair.first, pair.second));
	}
		
        // meta['description'] = "this wasn't great"    
        // meta['result_time'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())


	// and add some descriptive information
	mongo::BSONObjBuilder metaBuilder;
	metaBuilder.append("description", "this wasn\'t great, was it?");
	metaBuilder.append("result_time", mongo::Date_t(ros::Time::now().toSec()));

	// and store
    messageStore.insert(spl, metaBuilder.obj());

    // now let's get all our logged data back
	vector< boost::shared_ptr<StringPairList> > results;
	messageStore.query<StringPairList>(results);
       
	// note, as it stands cpp clients can't get the meta back, but it can be added if required


	for(auto & message : results) {

		// Hmmm... this code is ugly

		ROS_INFO_STREAM("Got data back");

		vector< boost::shared_ptr<Pose> > poses;
		messageStore.queryID<Pose>(message->pairs[0].second, poses);

		vector< boost::shared_ptr<Point> > points;
		messageStore.queryID<Point>(message->pairs[1].second, points);
		
		vector< boost::shared_ptr<Quaternion> > quaternions;
		messageStore.queryID<Quaternion>(message->pairs[2].second, quaternions);
		
		vector< boost::shared_ptr<Bool> > bools;
		messageStore.queryID<Bool>(message->pairs[3].second, bools);

	}       

	return 0;
}


