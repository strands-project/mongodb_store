
/***************************************************************************
 *  rosmongolog_pcl.cpp - MongoDB Logger for point cloud topics
 *
 *  Created: Mon Dec 13 15:05:37 2010
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

#include <sensor_msgs/PointCloud.h>
#include <unistd.h>

using namespace mongo;

DBClientConnection *mongodb_conn;
unsigned int counter;
std::string collection;

void msg_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  BSONObjBuilder document;
  BSONObjBuilder transform;

  
  const sensor_msgs::PointCloud& msg_in = *msg;

  Date_t stamp = msg_in.header.stamp.sec * 1000 + msg_in.header.stamp.nsec / 1000000;
  document.append("header", BSON("seq" << msg_in.header.seq
				       << "stamp" << stamp
				       << "frame_id" << msg_in.header.frame_id));

  BSONArrayBuilder pointb(document.subarrayStart("points"));
  // I can't believe it's encoded that inefficient. Someone should be sent to jail for that.
  std::vector<geometry_msgs::Point32>::const_iterator p;
  for (p = msg_in.points.begin(); p != msg_in.points.end(); ++p) {
    pointb.append(BSON(   "x" << p->x
		       << "y" << p->y
		       << "z" << p->z));
  }
  pointb.doneFast();

  BSONArrayBuilder channelsb(document.subarrayStart("channels"));
  BSONObjBuilder cb;

  std::vector<sensor_msgs::ChannelFloat32>::const_iterator c;
  for (c = msg_in.channels.begin(); c != msg_in.channels.end(); ++c) {
    cb.append("name", c->name);

    BSONArrayBuilder valuesb(cb.subarrayStart("values"));
    std::vector<float>::const_iterator v;
    for (v = c->values.begin(); v != c->values.end(); ++v) {
      valuesb.append(*v);
    }
    valuesb.doneFast();

    channelsb.append(cb.obj());
  }
  channelsb.doneFast();

  mongodb_conn->insert(collection, document.obj());
  ++counter;
}

void print_count(const ros::TimerEvent &te)
{
  ROS_INFO("Logged %u messages", counter);
}


int
main(int argc, char **argv)
{
  std::string topic = "", mongodb = "localhost", nodename = "";
  collection = "";

  int c;
  while ((c = getopt(argc, argv, "t:m:n:c:")) != -1) {
    if ((c == '?') || (c == ':')) {
      printf("Usage: %s -t topic -m mongodb -n nodename -c collection\n", argv[0]);
      exit(-1);
    } else if (c == 't') {
      topic = optarg;
    } else if (c == 'm') {
      mongodb = optarg;
    } else if (c == 'n') {
      nodename = optarg;
    } else if (c == 'n') {
      nodename = optarg;
    }
  }

  if (topic == "") {
    printf("No topic given.\n");
    exit(-2);
  } else if (nodename == "") {
    printf("No node name given.\n");
    exit(-2);
  }

  ros::init(argc, argv, nodename);
  ros::NodeHandle n;

  std::string errmsg;
  mongodb_conn = new DBClientConnection(/* auto reconnect*/ true);
  if (! mongodb_conn->connect(mongodb, errmsg)) {
    ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    return -1;
  }

  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud>(topic, 1000, msg_callback);
  ros::Timer count_print_timer = n.createTimer(ros::Duration(5, 0), print_count);

  ros::spin();

  delete mongodb_conn;

  return 0;
}
