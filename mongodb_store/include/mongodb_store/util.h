#ifndef __MONGODB_STORE_UTIL_H
#define __MONGODB_STORE_UTIL_H

#include <ros/ros.h>


//include to get BSON. There's probably a much smaller of set of headers we could get away with
#include "mongo/client/dbclient.h"


/*
if ctime is not None:


needs to be in native datetime format for language
    meta['inserted_at'] = datetime.utcfromtimestamp(ctime)
OR
    meta['inserted_at'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())

could allow addition meta if desirable
doc["_meta"]=meta

    #  also store type information

add stored class:
    doc["_meta"]["stored_class"] = msg.__module__ + "." + msg.__class__.__name__
	e.g. "nav_msgs.msg._OccupancyGrid.OccupancyGrid"

and type:
    doc["_meta"]["stored_type"] = msg._type
	e.g. "nav_msgs/OccupancyGrid"

*/

namespace mongodb_store {

template<typename MsgType> 
void add_meta_for_msg(const typename MsgType::ConstPtr & _msg,  mongo::BSONObjBuilder & _builder) {




	// doc["_meta"]["inserted_at"]
	// ros::Time::now()
	// Date_t stamp = msg_in.header.stamp.sec * 1000 + msg_in.header.stamp.nsec / 1000000;
	// AND FIX OTHER FILES WHEN SOLVED

	// doc["_meta"]["stored_type"]
	// ros::message_traits::DataType<MsgType>::value();

	// and the other will do 
	// http://www.cplusplus.com/reference/string/string/find/


}

}

#endif