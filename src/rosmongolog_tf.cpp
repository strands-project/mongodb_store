
/***************************************************************************
 *  rosmongolog_tf.cpp - MongoDB Logger for /tf
 *
 *  Created: Wed Dec 8 17:00:25 2010 -0500
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
 *             2010  Carnegie Mellon University
 *             2010  Intel Labs Pittsburgh
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <ros/ros.h>
#include <mongo/client/dbclient.h>

#include <tf/tfMessage.h>

#define COLLECTION "roslog.tf"
#define HOST "localhost"

using namespace mongo;

DBClientConnection *mongodb_conn;
unsigned int counter;

void msg_callback(const tf::tfMessage::ConstPtr& msg)
{
  std::vector<BSONObj> transforms;
  BSONObjBuilder transform_stamped;
  BSONObjBuilder transform;

  
  const tf::tfMessage& msg_in = *msg;

  std::vector<geometry_msgs::TransformStamped>::const_iterator t;
  for (t = msg_in.transforms.begin(); t != msg_in.transforms.end(); ++t) {
    Date_t stamp = t->header.stamp.sec * 1000 + t->header.stamp.nsec / 1000000;
    transform_stamped.append("header", BSON("seq" << t->header.seq
					    << "stamp" << stamp
					    << "frame_id" << t->header.frame_id));
    transform_stamped.append("child_frame_id", t->child_frame_id);
    transform.append("translation", BSON(   "x" << t->transform.translation.x
					 << "y" << t->transform.translation.y
					 << "z" << t->transform.translation.z));
    transform.append("rotation", BSON(   "x" << t->transform.rotation.x
				      << "y" << t->transform.rotation.y
				      << "z" << t->transform.rotation.z
				      << "w" << t->transform.rotation.w));
    transform_stamped.append("transform", transform.obj());
    transforms.push_back(transform_stamped.obj());
  }

  mongodb_conn->insert(COLLECTION, BSON("transforms" << transforms));
  ++counter;
}

void print_count(const ros::TimerEvent &te)
{
  ROS_INFO("Logged %u messages", counter);
}


int
main(int argc, char **argv)
{
  ros::init(argc, argv, "tfmongolog");
  ros::NodeHandle n;

  std::string errmsg;
  mongodb_conn = new DBClientConnection(/* auto reconnect*/ true);
  if (! mongodb_conn->connect(HOST, errmsg)) {
    ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    return -1;
  }

  ros::Subscriber sub = n.subscribe("tf", 1000, msg_callback);
  ros::Timer count_print_timer = n.createTimer(ros::Duration(5, 0), print_count);

  ROS_INFO("Logging started");
  ros::spin();

  delete mongodb_conn;

  return 0;
}
