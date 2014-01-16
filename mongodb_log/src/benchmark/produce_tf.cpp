
/***************************************************************************
 *  produce_tf.cpp - Produce tf messages for synthetic benchmark
 *
 *  Created: Fri Jul 13 17:25:45 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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
#include <tf/tfMessage.h>

ros::Publisher pub;

void
send_tf(const ros::TimerEvent &te)
{
  tf::tfMessage msg;
  geometry_msgs::TransformStamped ts;
  ts.header.frame_id = "/from";
  ts.child_frame_id = "/some_other";
  ts.transform.translation.x = 1.;
  ts.transform.rotation.w = 1.;

  msg.transforms.resize(5, ts);
  pub.publish(msg);
}

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "produce_tf");
  ros::NodeHandle n;

  std::string errmsg;

  pub = n.advertise<tf::tfMessage>("/tf", 10);
  ros::Timer send_timer = n.createTimer(ros::Duration(0, 10000000), send_tf);

  ros::spin();

  return 0;
}
