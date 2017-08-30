#include "mongodb_store/message_store.h"
#include "geometry_msgs/Pose.h"
#include "topic_tools/shape_shifter.h"

#include <boost/foreach.hpp>
#include <gtest/gtest.h>


#include <sstream>

using namespace geometry_msgs;
using namespace mongodb_store;
using namespace std;

// Declare a test
TEST(ROSDatacentre, cppTest)
{
	ros::NodeHandle nh;

	//Create object which does the work for us.
	MessageStoreProxy messageStore(nh, "test_collection");

	//This is the message we want to store
	Pose stored;
	string name("__this__is__a__cpp__test__pose__");
	//Insert something with a name
	string id(messageStore.insertNamed(name, stored));

	vector< boost::shared_ptr<Pose> > results;

	//Get it back, by default get one
	if(messageStore.queryNamed<Pose>(name, results)) {
		EXPECT_EQ(1, results.size());
		EXPECT_EQ(stored.position.x, results[0]->position.x);
        EXPECT_EQ(stored.position.y, results[0]->position.y);
        EXPECT_EQ(stored.position.z, results[0]->position.z);
        EXPECT_EQ(stored.orientation.x, results[0]->orientation.x);
        EXPECT_EQ(stored.orientation.y, results[0]->orientation.y);
        EXPECT_EQ(stored.orientation.z, results[0]->orientation.z);
        EXPECT_EQ(stored.orientation.w, results[0]->orientation.w);
	}
	else {
		ADD_FAILURE() << "Name query didn't find: " << name;
	}

	if(messageStore.queryID<Pose>(id, results)) {

		BOOST_FOREACH( boost::shared_ptr<Pose> p,  results)
		{
			ROS_INFO_STREAM("Got by ID: " << *p);
		}

	}


	stored.position.x = 999;
	messageStore.updateNamed(name, stored);
	//check the update worked
	results.clear();
	if(messageStore.queryNamed<Pose>(name, results)) {
		EXPECT_EQ(1, results.size());
		EXPECT_EQ(stored.position.x, results[0]->position.x);
        EXPECT_EQ(stored.position.y, results[0]->position.y);
        EXPECT_EQ(stored.position.z, results[0]->position.z);
        EXPECT_EQ(stored.orientation.x, results[0]->orientation.x);
        EXPECT_EQ(stored.orientation.y, results[0]->orientation.y);
        EXPECT_EQ(stored.orientation.z, results[0]->orientation.z);
        EXPECT_EQ(stored.orientation.w, results[0]->orientation.w);
	}
	else {
		ADD_FAILURE() << "Name query didn't find: " << name;
	}


	//check id query
	results.clear();
	if(messageStore.queryID<Pose>(id, results)) {
		EXPECT_EQ(1, results.size());
		EXPECT_EQ(stored.position.x, results[0]->position.x);
        EXPECT_EQ(stored.position.y, results[0]->position.y);
        EXPECT_EQ(stored.position.z, results[0]->position.z);
        EXPECT_EQ(stored.orientation.x, results[0]->orientation.x);
        EXPECT_EQ(stored.orientation.y, results[0]->orientation.y);
        EXPECT_EQ(stored.orientation.z, results[0]->orientation.z);
        EXPECT_EQ(stored.orientation.w, results[0]->orientation.w);
	}
	else {
		ADD_FAILURE() << "ID query didn't find: " << id;
	}

	// try to get it back with an incorrect name, so get None instead	
	std::string wrongName(name + "some garbage addition");
	results.clear();
	if(messageStore.queryNamed<Pose>(wrongName, results)) {
		ADD_FAILURE() << "Name query shouldn't have found: " << wrongName;
	}
	else {
		EXPECT_EQ(0, results.size());
	}


	// get all poses, should be at least one
	results.clear();
	EXPECT_TRUE(messageStore.query<Pose>(results));
	EXPECT_GT(results.size(), 0);


	// Now delete the thing we added before
	messageStore.deleteID(id);

	results.clear();
	if(messageStore.queryID<Pose>(id, results)) {
		ADD_FAILURE() << "ID query shouldn't have found " << id << " after deletion";
	}
	else {
		EXPECT_EQ(0, results.size());
	}

	results.clear();
	if(messageStore.queryNamed<Pose>(name, results)) {
		ADD_FAILURE() << "Name query shouldn't have found " << name << " after deletion";
	}
	else {
		EXPECT_EQ(0, results.size());
	}

    results.clear();
    for(int i = 0; i < 100; ++i) {
      geometry_msgs::Pose p;
      p.orientation.z = i;
      messageStore.insert<Pose>(p);
    }
    if(messageStore.query<Pose>(results, mongo::BSONObj(), mongo::BSONObj(), mongo::BSONObj(),false, 10)){
      EXPECT_EQ(10, results.size());
    }
    else {
      ADD_FAILURE() << "Documents are not limited";
    }
    results.clear();
    mongo::BSONObjBuilder builder;
    builder.append("orientation",0);
    if(messageStore.queryWithProjection<Pose>(results, mongo::BSONObj(), mongo::BSONObj(), mongo::BSONObj(), builder.obj(),false, 10)){
      EXPECT_EQ(0,results[1]->orientation.z);
    }
    else {
      ADD_FAILURE() << "Projection is not working correctly";
    }

    // non-wait insert
    unsigned int msg_num_before_insert = 0, msg_num_after_insert = 0;
    const std::string no_wait_name = "no_wait";
    results.clear();
    // query should not found before insert
    if (messageStore.queryNamed<Pose>(no_wait_name, results, false)) {
      ADD_FAILURE() << "failed query before insert no wait";
    }

    msg_num_before_insert = results.size();
    for (int i = 0; i < 10; ++i) {
      geometry_msgs::Pose p;
      p.orientation.x = i;
      messageStore.insertNamed<Pose>(no_wait_name, p, mongo::BSONObj(), /* wait = */false);
    }
    ros::Duration(2.0).sleep();
    results.clear();
    if (!messageStore.queryNamed<Pose>(no_wait_name, results, false)) {
      ADD_FAILURE() << "failed query after insert no wait";
    }
    msg_num_after_insert = results.size();
    EXPECT_GT(msg_num_after_insert, msg_num_before_insert);

    // Using topic_tools::ShapeShifter
    stored.position.x = 1;
    stored.position.y = 2;
    stored.position.z = 3;

    // converting geometry_msgs/Pose message into topic_tools/ShapeShifter type
    uint32_t serial_size = ros::serialization::serializationLength(stored);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream ostream(buffer.get(), serial_size);
    ros::serialization::serialize(ostream, stored);
    topic_tools::ShapeShifter ss_msg;
    ss_msg.morph(ros::message_traits::md5sum(stored),
                 ros::message_traits::datatype(stored),
                 ros::message_traits::definition(stored),
                 "0" /* = latch: false */);
    ros::serialization::IStream istream(buffer.get(), serial_size);
    ss_msg.read(istream);

    id = messageStore.insert(ss_msg);
    if (id.empty()) {
      ADD_FAILURE() << "Failed to insert message of type 'topic_tools::ShapeShifter'";
    }
    ROS_INFO_STREAM("Inserted message type of 'topic_tools::ShapeShifter': " << id);

    results.clear();
    if (!messageStore.queryID(id, results)) {
      ADD_FAILURE() << "Failed to query after insertion of 'topic_tools::ShapeShifter' message";
    }
    EXPECT_EQ(1, results.size());
    EXPECT_EQ(stored.position.x, results[0]->position.x);
    EXPECT_EQ(stored.position.y, results[0]->position.y);
    EXPECT_EQ(stored.position.z, results[0]->position.z);
    EXPECT_EQ(stored.orientation.x, results[0]->orientation.x);
    EXPECT_EQ(stored.orientation.y, results[0]->orientation.y);
    EXPECT_EQ(stored.orientation.z, results[0]->orientation.z);
    EXPECT_EQ(stored.orientation.w, results[0]->orientation.w);

    ROS_INFO_STREAM("happy here");
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "example_mongodb_store_cpp_client");
	return RUN_ALL_TESTS();
}


