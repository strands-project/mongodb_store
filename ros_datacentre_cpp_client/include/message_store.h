#ifndef __MESSAGE_STORE__H
#define __MESSAGE_STORE__H

#include "ros/ros.h"
#include "ros/console.h"
#include "ros_datacentre_msgs/MongoInsertMsg.h"
#include "ros_datacentre_msgs/MongoUpdateMsg.h"
#include "ros_datacentre_msgs/MongoQueryMsg.h"
#include "ros_datacentre_msgs/MongoDeleteMsg.h"
#include "ros_datacentre_msgs/StringPair.h"
#include "ros_datacentre_msgs/SerialisedMessage.h"

//include to get BSON. There's probably a much smaller of set of headers we could get away with
#include "mongo/client/dbclient.h"

#include  <boost/make_shared.hpp> 




namespace ros_datacentre {

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
	
	// std::cout<<"serial_size: "<<serial_size<<std::endl;	

 	//set msg vector to this size + 4 to hold length description
	_sm.msg.resize(serial_size);
	//serialise the object into the vector via this stream
	ros::serialization::OStream stream(&(_sm.msg[0]), serial_size);
	
	// serialise message
	ros::serialization::serialize(stream, _msg);
}


template<typename MsgType> 
boost::shared_ptr<MsgType> deserialise_message(ros_datacentre_msgs::SerialisedMessage & _sm) {

	// std::cout<<_sm<<std::endl;

	boost::shared_ptr<MsgType> message = boost::make_shared<MsgType>();

	uint32_t serial_size = _sm.msg.size();
	
	// std::cout<<"serial_size: "<<serial_size<<std::endl;	

	ros::serialization::IStream msgStream(&(_sm.msg[0]), serial_size);
	ros::serialization::deserialize(msgStream, *message);

	return message;
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
		m_deleteClient(handle.serviceClient<ros_datacentre_msgs::MongoDeleteMsg>(_servicePrefix + "/delete")),
		m_database(_database),
		m_collection(_collection)
	{

		m_insertClient.waitForExistence();
		m_updateClient.waitForExistence();
		m_queryClient.waitForExistence();
		m_deleteClient.waitForExistence();
	}

	MessageStoreProxy(const MessageStoreProxy& _rhs) :
		m_database(_rhs.m_database),
		m_collection(_rhs.m_collection),
		m_insertClient(_rhs.m_insertClient),
		m_updateClient(_rhs.m_insertClient),
		m_queryClient(_rhs.m_queryClient),
		m_deleteClient(_rhs.m_deleteClient)
	{}


	~MessageStoreProxy() {}

    ros_datacentre_msgs::StringPair makePair(const std::string & _first, const std::string & _second)
    {
        ros_datacentre_msgs::StringPair pair;
        pair.first = _first;
        pair.second = _second;
        return pair;
    }


	template<typename MsgType> 
	std::string insert(const MsgType & _msg) {
		return insert(_msg, m_database, m_collection);
	}


	template<typename MsgType> 
	std::string insertNamed(const std::string & _name, const MsgType & _msg, 
		const mongo::BSONObj & _meta = mongo::BSONObj()) {

		//create a copy of the meta data with the name included
		mongo::BSONObjBuilder builder;
		builder.appendElements(_meta);
		builder.append("name", _name);

		//and insert as usual
		return insert(_msg, m_database, m_collection, builder.obj());
	}

	template<typename MsgType> 
	std::string insert(const MsgType & _msg, 
		const std::string & _database, 
		const std::string & _collection, 
		const mongo::BSONObj & _meta = mongo::BSONObj()) {

  		//Create message with basic fields
  		ros_datacentre_msgs::MongoInsertMsg srv;
  		srv.request.database = _database;
  		srv.request.collection = _collection;
  		
 		
 		//if there's no meta then no copying is necessary
  		if(!_meta.isEmpty()) {
 			srv.request.meta.pairs.push_back(makePair(ros_datacentre_msgs::MongoQueryMsgRequest::JSON_QUERY, _meta.jsonString()));
		}

	 	fill_serialised_message(srv.request.message, _msg);

  		//sent data over
  		m_insertClient.call(srv);
  		return srv.response.id;

	}

	template<typename MsgType> 
	bool queryNamed(const std::string & _name, 
					std::vector< boost::shared_ptr<MsgType> > & _results, 
					bool _find_one = true) {

		
		mongo::BSONObj meta_query = BSON( "name" << _name );
		return query<MsgType>(_results, EMPTY_BSON_OBJ, meta_query, _find_one);
	}

	template<typename MsgType> 
	bool queryID(const std::string & _id, 
					std::vector< boost::shared_ptr<MsgType> > & _results) {
		
		mongo::BSONObj msg_query = BSON( "_id" << mongo::OID(_id) );
		return query<MsgType>(_results, msg_query, EMPTY_BSON_OBJ, true);
	}

	template<typename MsgType> 
	bool query(std::vector< boost::shared_ptr<MsgType> > & _results,
				const mongo::BSONObj & _message_query = mongo::BSONObj(),
				const mongo::BSONObj & _meta_query = mongo::BSONObj(),
				bool _find_one = false) {

		//Create message with basic fields
  		ros_datacentre_msgs::MongoQueryMsg msg;
  		msg.request.database = m_database;
  		msg.request.collection = m_collection;
  		msg.request.type = ros::message_traits::DataType<MsgType>::value();
  		msg.request.single = _find_one;
  	
		//if there's no message then no copying is necessary
  		if(!_message_query.isEmpty()) {
 			msg.request.message_query.pairs.push_back(makePair(ros_datacentre_msgs::MongoQueryMsgRequest::JSON_QUERY, _message_query.jsonString())); 			 			
		}

		//if there's no meta then no copying is necessary
		if(!_meta_query.isEmpty()) {
 			msg.request.meta_query.pairs.push_back(makePair(ros_datacentre_msgs::MongoQueryMsgRequest::JSON_QUERY, _meta_query.jsonString())); 			 			
		}

  		if(m_queryClient.call(msg)) {
  			ROS_INFO("Got back %li messages", msg.response.messages.size());
  			if(msg.response.messages.size() > 0) {
	  			for(size_t i = 0; i < msg.response.messages.size(); i ++) {
	  				_results.push_back(deserialise_message<MsgType>(msg.response.messages[i]));
	  			}			  		
	  			return true;
	  		}
  		}

		return false;

	}


	template<typename MsgType> 
	bool updateNamed(const std::string & _name, 
					const MsgType & _msg, 
					bool _upsert = false,
					const mongo::BSONObj & _meta = mongo::BSONObj()) {

		mongo::BSONObj meta_query = BSON( "name" << _name );

		// make sure the name goes into the meta info after update
		mongo::BSONObjBuilder builder;
		builder.appendElements(_meta);
		builder.append("name", _name);

		return update<MsgType>(_msg, builder.obj(), EMPTY_BSON_OBJ, meta_query, _upsert);
	}

	template<typename MsgType> 
	bool update(const MsgType & _msg, 
				const mongo::BSONObj & _meta = mongo::BSONObj(),
				const mongo::BSONObj & _message_query = mongo::BSONObj(),
				const mongo::BSONObj & _meta_query = mongo::BSONObj(),
				bool _upsert = false) {

		//Create message with basic fields
  		ros_datacentre_msgs::MongoUpdateMsg msg;
  		msg.request.database = m_database;
  		msg.request.collection = m_collection;
  		msg.request.upsert = _upsert;
  	
		//if there's no message then no copying is necessary
  		if(!_message_query.isEmpty()) {
 			msg.request.message_query.pairs.push_back(makePair(ros_datacentre_msgs::MongoQueryMsgRequest::JSON_QUERY, _message_query.jsonString())); 			 			
		}

		//if there's no meta then no copying is necessary
		if(!_meta_query.isEmpty()) {
 			msg.request.meta_query.pairs.push_back(makePair(ros_datacentre_msgs::MongoQueryMsgRequest::JSON_QUERY, _meta_query.jsonString())); 			 			
		}

		fill_serialised_message(msg.request.message, _msg);

			//if there's no meta then no copying is necessary
  		if(!_meta.isEmpty()) {
 			msg.request.meta.pairs.push_back(makePair(ros_datacentre_msgs::MongoQueryMsgRequest::JSON_QUERY, _meta.jsonString()));
		}


  		if(m_updateClient.call(msg)) {
  			return msg.response.success;
  		}
  		else {
 	 		return false;
 	 	}


	}

	bool deleteID(const std::string & _id) {

  		//Create message with basic fields
  		ros_datacentre_msgs::MongoDeleteMsg srv;
  		srv.request.database = m_database;
  		srv.request.collection = m_collection;
  		srv.request.document_id = _id;
  		 		
  		//sent data over
  		m_deleteClient.call(srv);
  		return srv.response.success;
	}



protected:


	std::string m_database;
	std::string m_collection;
	ros::ServiceClient m_insertClient;
	ros::ServiceClient m_updateClient;
	ros::ServiceClient m_queryClient;
	ros::ServiceClient m_deleteClient;

	//an empty bson doc to save recreating one whenever one is not required
    static const mongo::BSONObj EMPTY_BSON_OBJ;
};

}


#endif
