
#include "ros/ros.h"
#include "ros/console.h"
#include "ros_datacentre_msgs/MongoInsertMsg.h"
#include "ros_datacentre_msgs/MongoUpdateMsg.h"
#include "ros_datacentre_msgs/MongoQueryMsg.h"
#include "ros_datacentre_msgs/StringPair.h"
#include "ros_datacentre_msgs/SerialisedMessage.h"
#include  <boost/make_shared.hpp> 

namespace ros_datacentre {

ros_datacentre_msgs::StringPair makePair(const std::string & _first, const std::string & _second) {
	ros_datacentre_msgs::StringPair pair;
	pair.first = _first;
	pair.second = _second;
	return pair;
}


typedef std::vector<ros_datacentre_msgs::StringPair> StringPairs;


/**
Populates a SerialisedMessage using the given instance of MsgType
**/
template<typename MsgType> 
void fill_serialised_message(ros_datacentre_msgs::SerialisedMessage & _sm, 
								const MsgType & _msg) {
	
	//record type
	_sm.type = ros::message_traits::DataType<MsgType>::value();

	//how long the data will be
	uint32_t serial_size = ros::serialization::serializationLength(_msg);
 	//set msg vector to this size
	_sm.msg.resize(serial_size);
	//serialise the object into the vector via this stream
	ros::serialization::OStream stream(&(_sm.msg[0]), serial_size);
	ros::serialization::serialize(stream, _msg);

}


class MessageStoreProxy
{
public:

	/**

	**/
	MessageStoreProxy(ros::NodeHandle handle, 
		const std::string & _servicePrefix = "/message_store", 
		const std::string & _database = "message_store", 
		const std::string & _collection = "message_store") :
		m_insertClient(handle.serviceClient<ros_datacentre_msgs::MongoInsertMsg>(_servicePrefix + "/insert")),
		m_updateClient(handle.serviceClient<ros_datacentre_msgs::MongoUpdateMsg>(_servicePrefix + "/update")),
		m_queryClient(handle.serviceClient<ros_datacentre_msgs::MongoQueryMsg>(_servicePrefix + "/query_messages")),
		m_database(_database),
		m_collection(_collection)
	{

		m_insertClient.waitForExistence();
		m_updateClient.waitForExistence();
		m_queryClient.waitForExistence();
	}

	MessageStoreProxy(const MessageStoreProxy& _rhs) :
		m_database(_rhs.m_database),
		m_collection(_rhs.m_collection),
		m_insertClient(_rhs.m_insertClient),
		m_updateClient(_rhs.m_insertClient),
		m_queryClient(_rhs.m_queryClient)
	{}


	~MessageStoreProxy() {}


	template<typename MsgType> 
	void insert(const MsgType & _msg) {
		insert(_msg, m_database, m_collection);
	}


	template<typename MsgType> 
	void insertNamed(const std::string & _name, const MsgType & _msg, 
		const StringPairs & _meta = EMPTY_PAIR_LIST) {
		
		//make a copy so we can add stuff
		StringPairs meta = _meta;
		meta.push_back(makePair("name", _name));
		insert(_msg, m_database, m_collection, meta);
	}

	template<typename MsgType> 
	void insert(const MsgType & _msg, 
		const std::string & _database, 
		const std::string & _collection, 
		const StringPairs & _meta = EMPTY_PAIR_LIST) {

  		//Create message with basic fields
  		ros_datacentre_msgs::MongoInsertMsg msg;
  		msg.request.database = _database;
  		msg.request.collection = _collection;
  		
 		
 		//if there's no meta then no copying is necessary
  		if(_meta.size() > 0) {
 			msg.request.meta = _meta;
		}

	 	fill_serialised_message(msg.request.message, _msg);

  		//sent data over
  		m_insertClient.call(msg);
	}

	template<typename MsgType> 
	bool queryNamed(const std::string & _name, 
					std::vector< boost::shared_ptr<MsgType> > & _results, 
					bool find_one = true) {

		StringPairs meta_query;
		meta_query.push_back(makePair("name", _name));
		return query<MsgType>(_results, EMPTY_PAIR_LIST, meta_query, find_one);
	}

	template<typename MsgType> 
	bool query(std::vector< boost::shared_ptr<MsgType> > & _results,
				const StringPairs & _message_query = EMPTY_PAIR_LIST,
				const StringPairs & _meta_query = EMPTY_PAIR_LIST,
				bool find_one = false) {

		//Create message with basic fields
  		ros_datacentre_msgs::MongoQueryMsg msg;
  		msg.request.database = m_database;
  		msg.request.collection = m_collection;
  		msg.request.type = ros::message_traits::DataType<MsgType>::value();
  		msg.request.single = find_one;
  	
		//if there's no message then no copying is necessary
  		if(_message_query.size() > 0) {
 			msg.request.message_query.pairs = _message_query;
		}

		//if there's no meta then no copying is necessary
  		if(_meta_query.size() > 0) {
 			msg.request.meta_query.pairs = _meta_query;
		}

  		if(m_queryClient.call(msg)) {
  			ROS_INFO("Got back %li messages", msg.response.messages.size());
  			for(size_t i = 0; i < msg.response.messages.size(); i ++) {
  				_results.push_back(deserialise_message<MsgType>(msg.response.messages[i]));
  			}
  			return true;
  		}
  		else {
 	 		return false;
 	 	}

	}



	template<typename MsgType> 
	bool update(const MsgType & _msg, 
				const StringPairs & _meta = EMPTY_PAIR_LIST,
				const StringPairs & _message_query = EMPTY_PAIR_LIST,
				const StringPairs & _meta_query = EMPTY_PAIR_LIST,
				bool _upsert = false) {

		//Create message with basic fields
  		ros_datacentre_msgs::MongoUpdateMsg msg;
  		msg.request.database = m_database;
  		msg.request.collection = m_collection;
  		msg.request.upsert = _upsert;
  	
		//if there's no message then no copying is necessary
  		if(_message_query.size() > 0) {
 			msg.request.message_query.pairs = _message_query;
		}

		//if there's no meta then no copying is necessary
  		if(_meta_query.size() > 0) {
 			msg.request.meta_query.pairs = _meta_query;
		}

		fill_serialised_message(msg.request.message, _msg);

		//if there's no meta then no copying is necessary
  		if(_meta.size() > 0) {
 			msg.request.meta.pairs = _meta;
		}

  		if(m_updateClient.call(msg)) {
  			// ROS_INFO("Got back %li messages", msg.response.messages.size());
  			// for(size_t i = 0; i < msg.response.messages.size(); i ++) {
  			// 	_results.push_back(deserialise_message<MsgType>(msg.response.messages[i]));
  			// }
  			return true;
  		}
  		else {
 	 		return false;
 	 	}


	}

protected:

	template<typename MsgType> 
	boost::shared_ptr<MsgType> deserialise_message(ros_datacentre_msgs::SerialisedMessage & _sm) {

		boost::shared_ptr<MsgType> message = boost::make_shared<MsgType>();

		uint32_t serial_size = ros::serialization::serializationLength(*message);
		ros::serialization::IStream stream(&(_sm.msg[0]), serial_size);
		ros::serialization::deserialize(stream, *message);

		return message;
	}

	std::string m_database;
	std::string m_collection;
	ros::ServiceClient m_insertClient;
	ros::ServiceClient m_updateClient;
	ros::ServiceClient m_queryClient;

	//an empty vector to save recreating one whenever meta info is not provided
	static const StringPairs EMPTY_PAIR_LIST;
};


const StringPairs MessageStoreProxy::EMPTY_PAIR_LIST =  StringPairs();







}