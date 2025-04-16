import json
import typing

import pymongo.collection
import rclpy
import rclpy.client
import rclpy.node
import rclpy.serialization
import rclpy.type_support
import rosidl_runtime_py.utilities
import yaml
from bson import json_util, Binary
from pymongo.errors import ConnectionFailure
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty

from mongodb_store_msgs.msg import SerialisedMessage
from mongodb_store_msgs.srv import MongoQueryMsg


def check_connection_to_mongod(
    parent_node: rclpy.node.Node, db_host, db_port, connection_string=None
) -> bool:
    """
    Check connection to mongod server

    :Returns:
        | bool : True on success, False if connection is not established.
    """
    if check_for_pymongo():
        try:
            # pymongo 3.X
            from pymongo import MongoClient

            if connection_string is None:
                client = MongoClient(db_host, db_port, connect=False)
            else:
                client = MongoClient(connection_string)
            result = client.admin.command("ismaster")
            return True
        except ConnectionFailure:
            if connection_string is None:
                parent_node.get_logger().error(
                    f"Could not connect to mongo server {db_host}:{db_port}\nMake sure mongod is launched on your specified host/port"
                )
            else:
                parent_node.get_logger().error(
                    f"Could not connect to mongo server {connection_string}\nMake sure mongod is launched on your specified host/port"
                )
            return False
    else:
        return False


def wait_for_mongo(parent_node: rclpy.node.Node, timeout=60, ns="/datacentre"):
    """
    Waits for the mongo server, as started through the mongodb_store/mongodb_server.py wrapper

    :Returns:
        | bool : True on success, False if server not even started.
    """
    # # Check that mongo is live, create connection
    service = ns + "/wait_ready"
    wait_client = parent_node.create_client(Empty, service)

    result, message = check_and_get_service_result_async(
        parent_node, wait_client, Empty.Request()
    )
    if result is None:
        parent_node.get_logger().error(
            "Can't connect to MongoDB server. Make sure mongodb_store/mongodb_server.py node is started."
        )
        return False
    return True


def check_and_get_service_result_async(
    node: rclpy.node.Node,
    client: rclpy.client.Client,
    request,
    existence_timeout: typing.Optional[float] = 1,
    spin_timeout: typing.Optional[float] = None,
) -> typing.Tuple[typing.Optional[typing.Any], str]:
    """
    Check the service for the given client exists, then call it and retrieve the response asynchronously,
    but block to do so. Calls the executor from the node associated with the client, using its
    spin_until_future_complete function

    Note: If the client is being called from a callback (i.e. subscriber callback, service callback, any actionserver
    callback) You must ensure that the client is in a separate callback group to the one which is initiating this
    call. If it is not, you will probably get a silent deadlock.

    Args:
        node: Node which created the client. TODO: Is this needed? The client has a context and handle but not sure if those have references to the node
        client: Client to call
        request: Request to send to the client
        existence_timeout: How long to wait for the service to become available
        spin_timeout: How long to spin waiting for the result before timing out

    Returns:
        Tuple with result of the call, or None if it failed, and a message
    """
    if not client.wait_for_service(timeout_sec=existence_timeout):
        message = f"Couldn't find {client.srv_name}"
        node.get_logger().error(message)
        return None, message
    resp_future = client.call_async(request)
    # Don't use rclpy.spin_until_future_completes on the node because then the node is removed from the global
    # executor and will not receive any further callbacks
    if not node.executor:
        # do this in case the node doesn't have an executor, and remove the executor after we're done, otherwise the
        # executor is permanently set as that node's executor
        rclpy.spin_until_future_complete(
            node,
            resp_future,
            timeout_sec=spin_timeout,
            executor=MultiThreadedExecutor(),
        )
        node.executor = None
    else:
        node.executor.spin_until_future_complete(resp_future, timeout_sec=spin_timeout)

    return resp_future.result(), f"Successfully called {client.srv_name}"


def check_for_pymongo():
    """
    Checks for required version of pymongo python library.

    :Returns:
        | bool : True if found, otherwise Fale
    """
    try:
        import pymongo
    except:
        print("ERROR!!!")
        print("Can't import pymongo, this is needed by mongodb_store.")
        print("Make sure it is installed (pip install pymongo)")
        return False

    return True


def import_MongoClient():
    """
    Pick an object to use as MongoClient based on the currently installed pymongo
    version. Use this instead of importing Connection or MongoClient from pymongo
    directly.

    Example:
        MongoClient = util.importMongoClient()
    """
    import pymongo

    if pymongo.version >= "2.4":

        def mongo_client_wrapper(*args, **kwargs):
            return pymongo.MongoClient(*args, **kwargs)

        return mongo_client_wrapper


def msg_to_document(msg):
    """
    Given a ROS message, turn it into a (nested) dictionary suitable for the datacentre.

    >>> from geometry_msgs.msg import Pose
    >>> msg_to_document(Pose())
    {'orientation': {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}}

    :Args:
        | msg (ROS Message): An instance of a ROS message to convert
    :Returns:
        | dict : A dictionary representation of the supplied message.
    """
    return yaml.safe_load(rosidl_runtime_py.message_to_yaml(msg))


def sanitize_value(attr, v, type):
    """
    De-rosify a msg.

    Internal function used to convert ROS messages into dictionaries of pymongo insertable
    values.

    :Args:
        | attr(str): the ROS message slot name the value came from
        | v: the value from the message's slot to make into a MongoDB able type
        | type (str): The ROS type of the value passed, as given by the ressage slot_types member.
    :Returns:
        | A sanitized version of v.
    """

    # print '---'
    # print attr
    # print v.__class__
    # print type
    # print v

    if isinstance(v, str):
        if type == "uint8[]":
            v = Binary(v)

        # no need to carry on with the other type checks below
        return v

    if rclpy.type_support.check_for_type_support(v):
        # This should be a sufficient check for whether something is a ros msg, srv, or action
        return msg_to_document(v)
    elif isinstance(v, list):
        result = []
        for t in v:
            if hasattr(t, "_type"):
                result.append(sanitize_value(None, t, t._type))
            else:
                result.append(sanitize_value(None, t, None))
        return result
    else:
        return v


def store_message(collection: pymongo.collection.Collection, msg, meta, oid=None):
    """
    Update ROS message into the DB

    :Args:
        | collection (pymongo.Collection): the collection to store the message in
        | msg (ROS message): an instance of a ROS message to store
        | meta (dict): Additional meta data to store with the ROS message
        | oid (str): An optional ObjectID for the MongoDB document created.
    :Returns:
        | str: ObjectId of the MongoDB document.
    """
    # The message should be stored separate from the meta fields so it can be more easily restored. Otherwise rosidl
    # dict to message has problems later because the message doesn't have the meta fields
    doc = {"message": msg_to_document(msg)}

    message_type = message_to_namespaced_type(msg)
    doc["_meta"] = meta
    #  also store type information
    doc["_meta"]["stored_class"] = ".".join([msg.__module__, msg.__class__.__name__])
    doc["_meta"]["stored_type"] = message_type

    if hasattr(msg, "_connection_header"):
        print(getattr(msg, "_connection_header"))

    if oid != None:
        doc["_id"] = oid

    return collection.insert_one(doc)


def store_message_no_meta(collection, msg):
    """
    Store a ROS message sans meta data.

    :Args:
        | collection (pymongo.Collection): The collection to store the message in
        | msg (ROS message): An instance of a ROS message to store
    :Returns:
        | str: The ObjectId of the MongoDB document created.
    """
    doc = msg_to_document(msg)
    return collection.insert_one(doc)


def query_message(
    collection,
    query_doc,
    sort_query=None,
    projection_query=None,
    find_one=False,
    limit=0,
):
    """
    Peform a query for a stored messages, returning results in list.

    :Args:
        | collection (pymongo.Collection): The collection to query
        | query_doc (dict): The MongoDB query to execute
        | sort_query (list of tuple): The MongoDB query to sort
        | projection_query (dict): The projection query
        | find_one (bool): Returns one matching document if True, otherwise all matching.
        | limit (int): Limits number of return documents. 0 means no limit
    :Returns:
        | dict or list of dict: the MongoDB document(s) found by the query
    """
    if sort_query is None:
        sort_query = []
    if projection_query is None:
        projection_query = {}
    if find_one:
        ids = ()
        if sort_query:
            if not projection_query:
                result = collection.find_one(query_doc, sort=sort_query)
            else:
                result = collection.find_one(
                    query_doc, projection_query, sort=sort_query
                )
        elif projection_query:
            result = collection.find_one(query_doc, projection_query)
        else:
            result = collection.find_one(query_doc)
        if result:
            return [result]
        else:
            return []
    else:
        if sort_query:
            if not projection_query:
                return [
                    result
                    for result in collection.find(query_doc)
                    .sort(sort_query)
                    .limit(limit)
                ]
            else:
                return [
                    result
                    for result in collection.find(query_doc, projection_query)
                    .sort(sort_query)
                    .limit(limit)
                ]
        elif projection_query:
            return [
                result
                for result in collection.find(query_doc, projection_query).limit(limit)
            ]
        else:
            return [result for result in collection.find(query_doc).limit(limit)]


def update_message(
    collection: pymongo.collection.Collection, query_doc, msg, meta, upsert
):
    """
    Update ROS message in the DB, return updated id and true if db altered.

    :Args:
        | collection (pymongo.Collection): The collection to update in
        | query_doc (dict): The MongoDB query to execute to select document for update
        | msg (ROS message): An instance of a ROS message to update to
        | meta (dict): New meta data to update the stored message with
        | upsert (bool): If message does not already exits, create if upsert==True.
    :Returns:
        | str, bool: the OjectId of the updated document and whether it was altered by
                     the operation
    """
    # see if it's in db first
    result = collection.find_one(query_doc)

    # if it's not in there but we're allowed to insert
    if not result:
        if upsert:
            return store_message(collection, msg, meta), True
        else:
            return "", False

    # convert msg to db document
    doc = {"message": msg_to_document(msg)}

    # update _meta
    doc["_meta"] = result["_meta"]
    # merge the two dicts, overwiriting elements in doc["_meta"] with elements in meta
    doc["_meta"] = dict(list(doc["_meta"].items()) + list(meta.items()))

    # ensure necessary parts are there too
    doc["_meta"]["stored_class"] = ".".join([msg.__module__, msg.__class__.__name__])
    doc["_meta"]["stored_type"] = message_to_namespaced_type(msg)

    # Have to use the $set command to actually update the matching entry
    set_cmd = {"$set": doc}

    return collection.update_one(query_doc, set_cmd), True


def query_message_ids(collection, query_doc, find_one):
    """
    Peform a query for a stored message, returning a tuple of id strings

    :Args:
        | collection (pymongo.Collection): The collection to search
        | query_doc (dict): The MongoDB query to execute
        | find_one (bool): Find one matching document if True, otherwise all matching.
    :Returns:
        | tuple of strings: all ObjectIds of matching documents
    """
    if find_one:
        result = collection.find_one(query_doc)
        if result:
            return (str(result["_id"]),)
    else:
        return tuple(
            str(result["_id"]) for result in collection.find(query_doc, {"_id": 1})
        )


def message_to_namespaced_type(message):
    """
    Takes a ROS msg and turn it into a namespaced type

    E.g
    >>> type(Pose())
    <class 'geometry_msgs.msg._pose.Pose'>
    >>> type_to_class_string(Pose())
    geometry_msgs/Pose

    :Args:
        | type (ROS message): The ROS message object
    :Returns:
        | str: A python class string for the ROS message type supplied
    """
    if "metaclass" in str(type(message)).lower():
        # print(
        #     f"Received message {type(message)} which is a metaclass. You should pass instantiated objects rather than "
        #     f"metaclasses, but I'll convert it for you."
        # )
        message = message()
    message_type = type(message)
    module_parts = message_type.__module__.split(".")
    cls_string = f"{module_parts[0]}/{module_parts[1]}/{message_type.__name__}"
    return cls_string


def serialise_message(message):
    """
    Create a mongodb_store_msgs/SerialisedMessage instance from a ROS message.

    :Args:
        | message (ROS message): The message to serialise
    :Returns:
        | mongodb_store_msgs.msg.SerialisedMessage: A serialised copy of message
    """
    msg_bytes = rclpy.serialization.serialize_message(message)
    serialised_msg = SerialisedMessage()
    serialised_msg.msg = msg_bytes
    serialised_msg.type = message_to_namespaced_type(message)

    return serialised_msg


def deserialise_message(serialised_message):
    """
    Create a ROS message from a mongodb_store_msgs/SerialisedMessage

    :Args:
        | serialised_message (mongodb_store_msgs.msg.SerialisedMessage): The message to deserialise
    :Returns:
        | ROS message: The message deserialised
    """
    cls = rosidl_runtime_py.utilities.get_interface(serialised_message.type)
    message = rclpy.serialization.deserialize_message(
        bytes(serialised_message.msg), cls
    )
    return message


def string_pair_list_to_dictionary_no_json(spl):
    """
    Covert a mongodb_store_msgs/StringPairList into a dictionary, ignoring content

    :Args:
        | spl (StringPairList): The list of (key, value) to pairs convert
    :Returns:
        | dict: resulting dictionary
    """
    return dict((pair.first, pair.second) for pair in spl)


def string_pair_list_to_dictionary(spl):
    """
    Creates a dictionary from a mongodb_store_msgs/StringPairList which could contain JSON as a string.
    If the first entry in the supplied list is a JSON query then the returned dictionary is loaded from that.

    :Args:
        | spl (StringPairList): The list of (key, value) pairs to convert
    :Returns:
        | dict: resulting dictionary
    """
    if len(spl.pairs) > 0 and spl.pairs[0].first == MongoQueryMsg.Request.JSON_QUERY:
        # print "looks like %s", spl.pairs[0].second
        # json loads will return None if the pair value is 'null'. Make sure it returns a dict.
        return json.loads(spl.pairs[0].second, object_hook=json_util.object_hook) or {}
    # else use the string pairs
    else:
        return string_pair_list_to_dictionary_no_json(spl.pairs)


def topic_name_to_collection_name(topic_name):
    """
    Converts the fully qualified name of a topic into legal mongodb collection name.
    """
    return topic_name.replace("/", "_")[1:]
