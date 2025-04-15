import copy
import json
import typing

import rclpy
from bson import json_util
from bson.objectid import ObjectId

import mongodb_store.util as dc_util
from mongodb_store_msgs.msg import StringPair, StringPairList, Insert
from mongodb_store_msgs.srv import (
    MongoInsertMsg,
    MongoDeleteMsg,
    MongoQueryMsg,
    MongoUpdateMsg,
)


class MessageStoreProxy:
    """
    A class that provides functions for storage and retrieval of ROS Message
    objects in the mongodb_store. This is achieved by acting as a proxy to the
    services provided by the MessageStore ROS node, and therefore requires the message
    store node to be running in addition to the datacentre:

    `rosrun mongodb_store message_store_node.py`

     >>> from geometry_msgs.msg import Pose, Quaternion
     >>> msg_store = MessageStoreProxy()
     >>> p = Pose(Point(0, 1, 2), Quaternion(0, 0, 0 , 1))
     >>> msg_store.insert_named("my favourite pose", p)
     >>> retrieved = msg_store.query_named("my favourite pose", Pose._type)

    For usage examples, please see `example_message_store_client.py` within the scripts
    folder of mongodb_store.

    """

    def __init__(
        self,
        parent_node: rclpy.node.Node,
        service_prefix="/message_store",
        database="message_store",
        collection="message_store",
        queue_size=100,
    ) -> None:
        """
        Args:
           | service_prefix (str): The prefix to the *insert*, *update*, *delete* and
             *query_messages* ROS services/
           | database (str): The MongoDB database that this object works with.
           | collection (str): The MongoDB collect/on that this object works with.
        """
        self.parent_node = parent_node
        self.database = database
        self.collection = collection
        insert_service = service_prefix + "/insert"
        update_service = service_prefix + "/update"
        delete_service = service_prefix + "/delete"
        query_service = service_prefix + "/query_messages"
        # try and get the mongo service, block until available
        found_services_first_try = True  # if found straight away
        self.insert_srv = self.parent_node.create_client(MongoInsertMsg, insert_service)
        self.update_srv = self.parent_node.create_client(MongoUpdateMsg, update_service)
        self.query_srv = self.parent_node.create_client(MongoQueryMsg, query_service)
        self.delete_srv = self.parent_node.create_client(MongoDeleteMsg, delete_service)

        insert_topic = service_prefix + "/insert"
        self.pub_insert = self.parent_node.create_publisher(Insert, insert_topic, 10)

        while rclpy.ok():
            try:
                self.insert_srv.wait_for_service(5)
                self.update_srv.wait_for_service(5)
                self.query_srv.wait_for_service(5)
                self.delete_srv.wait_for_service(5)
                break
            except Exception as e:
                found_services_first_try = False
                self.parent_node.get_logger().error(
                    "Could not get message store services. Maybe the message "
                    "store has not been started? Retrying..."
                )
        if not found_services_first_try:
            self.parent_node.get_logger().info("Message store services found.")

    def insert_named(
        self,
        name: str,
        message: "RosMessage",
        meta: typing.Dict = None,
        wait: bool = True,
    ) -> str:
        """
        Inserts a ROS message into the message storage, giving it a name for convenient
        later retrieval.
        .. note:: Multiple messages can be stored with the same name.

        :Args:
            | name (str): The name to refere to this message as.
            | message (ROS Message): An instance of a ROS message type to store
            | meta (dict): A dictionary of additional meta data to store in association
                                  with thie message.
            | wait (bool): If true, waits until database returns object id after insert
        :Returns:
            | (str) the ObjectId of the MongoDB document containing the stored message.
        """
        # create a copy as we're modifying it
        if meta is None:
            meta = {}
        meta_copy = copy.copy(meta)
        meta_copy["name"] = name
        return self.insert(message, meta_copy, wait=wait)

    def insert(
        self, message: "ROSMessage", meta: typing.Dict = None, wait: bool = True
    ) -> typing.Union[bool, str]:
        """
        Inserts a ROS message into the message storage.

        :Args:
            | message (ROS Message): An instance of a ROS message type to store
            | meta (dict): A dictionary of additional meta data to store in association
                                  with thie message.
            | wait (bool): If true, waits until database returns object id after insert
        :Returns:
            | (str) the ObjectId of the MongoDB document containing the stored message.

        """
        # assume meta is a dict, convert k/v to tuple pairs
        meta_tuple = (
            StringPair(
                first=MongoQueryMsg.Request.JSON_QUERY,
                second=json.dumps(meta, default=json_util.default),
            ),
        )
        serialised_msg = dc_util.serialise_message(message)
        request = MongoInsertMsg.Request()
        request.database = self.database
        request.collection = self.collection
        request.meta = StringPairList(pairs=meta_tuple)
        request.message = serialised_msg

        if wait:
            return dc_util.check_and_get_service_result_async(
                self.parent_node, self.insert_srv, request
            )[0].id
        else:
            msg = Insert(
                database=self.database,
                collection=self.collection,
                message=serialised_msg,
                meta=StringPairList(pairs=meta_tuple),
            )
            self.pub_insert.publish(msg)
            return True

    def query_id(self, id: str, type: str):
        """
        Finds and returns the message with the given ID.

        :Parameters:
            | id (str): The ObjectID of the MongoDB document holding the message.
            | type (str): The ROS message type of the stored messsage to retrieve.
        :Returns:
            | message (ROS message), meta (dict): The retrieved message and associated metadata
              or *None* if the named message could not be found.
        """
        return self.query(type, {"_id": ObjectId(id)}, {}, True)

    def delete(self, message_id: str) -> bool:
        """
        Delete the message with the given ID.

        :Parameters:
            | message_id (str) : The ObjectID of the MongoDB document holding the message.
        :Returns:
            | bool : was the object successfully deleted.
        """
        request = MongoDeleteMsg.Request()
        request.database = self.database
        request.collection = self.collection
        request.document_id = message_id
        return dc_util.check_and_get_service_result_async(
            self.parent_node, self.delete_srv, request
        )[0].success

    def query_named(
        self,
        name: str,
        type: str,
        single: bool = True,
        meta: typing.Dict = None,
        limit: int = 0,
    ):
        """
        Finds and returns the message(s) with the given name.

        :Args:
            | name (str): The name of the stored messages to retrieve.
            | type (str): The type of the stored message.
            | single (bool): Should only one message be returned?
            | meta (dict): Extra queries on the meta data of the message.
            | limit (int): Limit number of return documents
        :Return:
            | message (ROS message), meta (dict): The retrieved message and associated metadata
              or *None* if the named message could not be found.

        """
        # create a copy as we're modifying it
        if meta is None:
            meta = {}
        meta_copy = copy.copy(meta)
        meta_copy["name"] = name
        return self.query(
            type, {}, meta_copy, single, [], projection_query={}, limit=limit
        )

    def update_named(
        self,
        name: str,
        message: "ROSMessage",
        meta: typing.Dict = None,
        upsert: bool = False,
    ) -> typing.Tuple[str, bool]:
        """
        Updates a named message.

        :Args:
            | name (str): The name of the stored messages to update.
            | message (ROS Message): The updated ROS message
            | meta (dict): Updated meta data to store with the message.
            | upsert (bool): If True, insert the named message if it doesnt exist.
        :Return:
            | str, bool: The MongoDB ObjectID of the document, and whether it was altered by
                         the update.
        """
        meta_query = {}
        meta_query["name"] = name

        # make sure the name goes into the meta info after update
        if meta is None:
            meta = {}
        meta_copy = copy.copy(meta)
        meta_copy["name"] = name

        return self.update(message, meta_copy, {}, meta_query, upsert)

    def update_id(self, id, message, meta=None, upsert=False):
        """
        Updates a message by MongoDB ObjectId.

        Args:
            id: The MongoDB ObjectId of the doucment storing the message.
            message: The updated ROS message
            meta: Updated meta data to store with the message.
            upsert: If True, insert the named message if it doesnt exist.
        Return:
            str, bool: The MongoDB ObjectID of the document, and whether it was altered by
                         the update.

        """

        msg_query = {"_id": ObjectId(id)}
        meta_query = {}

        return self.update(message, meta, msg_query, meta_query, upsert)

    def update(
        self,
        message: "ROSMessage",
        meta: typing.Dict = None,
        message_query: typing.Dict = None,
        meta_query: typing.Dict = None,
        upsert: bool = False,
    ) -> MongoUpdateMsg.Response:
        """
        Updates a message.

        :Args:
            | message (ROS Message): The updated ROS message
            | meta (dict): Updated meta data to store with the message.
            | message_query (dict): A query to match the ROS message that is to be updated.
            | meta_query (dict): A query to match against the meta data of the message to be updated
            | upsert (bool): If True, insert the named message if it doesnt exist.
        :Return:
            | str, bool: The MongoDB ObjectID of the document, and whether it was altered by
                         the update.

        """
        if message_query is None:
            message_query = {}
        if meta_query is None:
            meta_query = {}
        if meta is None:
            meta = {}

        # serialise the json queries to strings using json_util.dumps
        message_query_tuple = (
            StringPair(
                first=MongoQueryMsg.Request.JSON_QUERY,
                second=json.dumps(message_query, default=json_util.default),
            ),
        )
        meta_query_tuple = (
            StringPair(
                first=MongoQueryMsg.Request.JSON_QUERY,
                second=json.dumps(meta_query, default=json_util.default),
            ),
        )
        meta_tuple = (
            StringPair(
                first=MongoQueryMsg.Request.JSON_QUERY,
                second=json.dumps(meta, default=json_util.default),
            ),
        )

        request = MongoUpdateMsg.Request()
        request.database = self.database
        request.collection = self.collection
        request.upsert = upsert
        request.message_query = StringPairList(pairs=message_query_tuple)
        request.meta_query = StringPairList(pairs=meta_query_tuple)
        request.message = dc_util.serialise_message(message)
        request.meta = StringPairList(pairs=meta_tuple)

        return dc_util.check_and_get_service_result_async(
            self.parent_node, self.update_srv, request
        )[0]

    def query(
        self,
        type: str,
        message_query: typing.Dict = None,
        meta_query: typing.Dict = None,
        single: bool = False,
        sort_query: typing.List[typing.Tuple] = None,
        projection_query: typing.Dict = None,
        limit: int = 0,
    ):
        """
        Finds and returns message(s) matching the message and meta data queries.

        :Parameters:
            | type (str): The ROS message type of the stored messsage to retrieve.
            | message_query (dict): A query to match the actual ROS message
            | meta_query (dict): A query to match against the meta data of the message
            | sort_query (list of tuple): A query to request sorted list to mongodb module
            | projection_query (dict): A query to request desired fields to be returned or excluded
            | single (bool): Should only one message be returned?
                    | limit (int): Limit number of return documents
        :Returns:
            | [message, meta] where message is the queried message and meta a dictionary of
              meta information. If single is false returns a list of these lists.
        """
        if message_query is None:
            message_query = {}
        if meta_query is None:
            meta_query = {}
        if sort_query is None:
            sort_query = []
        if projection_query is None:
            projection_query = {}

        # assume meta is a dict, convert k/v to tuple pairs for ROS msg type

        # serialise the json queries to strings using json_util.dumps

        message_tuple = StringPairList(
            pairs=[
                StringPair(
                    first=MongoQueryMsg.Request.JSON_QUERY,
                    second=json.dumps(message_query, default=json_util.default),
                ),
            ]
        )
        meta_tuple = StringPairList(
            pairs=[
                StringPair(
                    first=MongoQueryMsg.Request.JSON_QUERY,
                    second=json.dumps(meta_query, default=json_util.default),
                ),
            ]
        )
        projection_tuple = StringPairList(
            pairs=[
                StringPair(
                    first=MongoQueryMsg.Request.JSON_QUERY,
                    second=json.dumps(projection_query, default=json_util.default),
                ),
            ]
        )

        if len(sort_query) > 0:
            sort_tuple = StringPairList(
                pairs=[StringPair(first=str(k), second=str(v)) for k, v in sort_query]
            )
        else:
            sort_tuple = StringPairList()

        request = MongoQueryMsg.Request()
        request.database = self.database
        request.collection = self.collection
        request.type = type
        request.single = single
        request.limit = limit
        request.message_query = message_tuple
        request.meta_query = meta_tuple
        request.projection_query = projection_tuple
        request.sort_query = sort_tuple

        response = dc_util.check_and_get_service_result_async(
            self.parent_node, self.query_srv, request
        )[0]

        if response.messages is None:
            messages = []
            metas = []
        else:
            messages = list(map(dc_util.deserialise_message, response.messages))
            metas = list(map(dc_util.string_pair_list_to_dictionary, response.metas))

        if single:
            if len(messages) > 0:
                return [messages[0], metas[0]]
            else:
                return [None, None]
        else:
            return list(zip(messages, metas))
