#include "ros_datacentre/message_store.h"

namespace ros_datacentre {

	const mongo::BSONObj MessageStoreProxy::EMPTY_BSON_OBJ =  mongo::BSONObj();

	ros_datacentre_msgs::StringPair makePair(const std::string & _first, const std::string & _second) {
	    ros_datacentre_msgs::StringPair pair;
	    pair.first = _first;
	    pair.second = _second;
	    return pair;
	}

}