"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import rosidl_runtime_py
import rosidl_runtime_py.utilities
import json
from datetime import datetime, timezone

import pymongo
import rclpy
from bson import json_util
from bson.objectid import ObjectId
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor
from tf2_msgs.msg import TFMessage

import mongodb_store.util as dc_util
from mongodb_store_msgs.msg import StringPair, StringPairList, Insert
from mongodb_store_msgs.srv import (
    MongoQueryMsg,
    MongoUpdateMsg,
    MongoDeleteMsg,
    MongoInsertMsg,
    MongoQuerywithProjectionMsg,
)

MongoClient = dc_util.import_MongoClient()


class MessageStore(rclpy.node.Node):
    def __init__(self, replicate_on_write=False):
        super().__init__("message_store")
        use_daemon = self.declare_parameter(
            "mongodb_use_daemon",
            False,
            descriptor=ParameterDescriptor(description="Use the daemon"),
        ).value
        connection_string = self.declare_parameter(
            "mongodb_connection_string",
            "",
            descriptor=ParameterDescriptor(description=""),
        ).value
        use_connection_string = len(connection_string) > 0
        if use_connection_string:
            use_daemon = True
            self.get_logger().info("Using connection string: %s", connection_string)

        # If you want to use a remote datacenter, then it should be set as false
        use_localdatacenter = self.declare_parameter(
            "mongodb_use_localdatacenter", True
        )
        local_timeout = self.declare_parameter("local_timeout", 10).value
        if str(local_timeout).lower() == "none":
            local_timeout = None

        # wait for hostname and port for mongodb server
        # TODO: This is limited to a specific name for the server
        param_client = self.create_client(
            GetParameters, "/mongodb_server/get_parameters"
        )
        request = GetParameters.Request()
        request.names = ["mongodb_host", "mongodb_port"]

        result, message = dc_util.check_and_get_service_result_async(
            self, param_client, request, existence_timeout=10
        )
        if result is None:
            raise RuntimeError(
                f"Could not find service {param_client.srv_name} from which to retrieve mongo host and port parameters"
            )

        host_value = result.values[0]
        port_value = result.values[1]
        if host_value.type != ParameterType.PARAMETER_STRING:
            raise RuntimeError(
                f"Parameter value for the mongodb_host param was not a string: {host_value}. Check the /mongodb_server parameters."
            )
        if port_value.type != ParameterType.PARAMETER_INTEGER:
            raise RuntimeError(
                f"Parameter value for the mongodb_port param was not an integer: {port_value}. Check the /mongodb_server parameters."
            )

        mongodb_host = host_value.string_value
        mongodb_port = port_value.integer_value

        db_host = self.declare_parameter(
            "mongodb_host", mongodb_host, descriptor=ParameterDescriptor(description="")
        ).value
        db_port = self.declare_parameter(
            "mongodb_port", mongodb_port, descriptor=ParameterDescriptor(description="")
        ).value

        if use_daemon:
            if use_connection_string:
                is_daemon_alive = dc_util.check_connection_to_mongod(
                    self, None, None, connection_string=connection_string
                )
            else:
                is_daemon_alive = dc_util.check_connection_to_mongod(
                    self, db_host, db_port
                )
            if not is_daemon_alive:
                raise Exception("No Daemon?")
        elif use_localdatacenter:
            self.get_logger().info(
                f"Waiting for local datacentre (timeout: {local_timeout})"
            )
            have_dc = dc_util.wait_for_mongo(self, local_timeout)
            if not have_dc:
                raise Exception("No Datacentre?")
            else:
                self.get_logger().info("Got datacentre")

        self.keep_trash = self.declare_parameter(
            "mongodb_keep_trash", True, descriptor=ParameterDescriptor(description="")
        ).value

        if use_connection_string:
            self._mongo_client = MongoClient(connection_string)
        else:
            self._mongo_client = MongoClient(db_host, db_port)

        self.replicate_on_write = self.declare_parameter(
            "mongodb_replicate_on_write", replicate_on_write
        ).value
        if self.replicate_on_write:
            self.get_logger().warning(
                "The option 'replicate_on_write' is now deprecated and will not function. "
                "Use 'Replication' on MongoDB instead: "
                "https://docs.mongodb.com/manual/replication/"
            )

        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service = getattr(self, attr)
                self.create_service(
                    service.type, "/message_store/" + attr[:-8], service
                )

        self.queue_size = self.declare_parameter(
            "queue_size", 100, descriptor=ParameterDescriptor(description="")
        ).value
        self.sub_insert = self.create_subscription(
            Insert,
            "/message_store/insert",
            self.insert_ros_msg,
            self.queue_size,
        )

    def insert_ros_msg(self, msg):
        """
        Receives a message published
        """
        # actually procedure is the same
        self.insert_ros_srv(msg, MongoInsertMsg.Response())

    def insert_ros_srv(
        self, request: MongoInsertMsg.Request, response: MongoInsertMsg.Response
    ) -> MongoInsertMsg.Response:
        """
        Receives a
        """
        # deserialize data into object
        obj = dc_util.deserialise_message(request.message)
        # convert input tuple to dict
        meta = dc_util.string_pair_list_to_dictionary(request.meta)
        # get requested collection from the db, creating if necessary
        collection = self._mongo_client[request.database][request.collection]
        # check if the object has the location attribute
        if hasattr(obj, "pose"):
            # if it does create a location index
            collection.create_index([("loc", pymongo.GEO2D)])

        # check if the object has the location attribute
        if hasattr(obj, "geotype"):
            # if it does create a location index
            collection.create_index([("geoloc", pymongo.GEOSPHERE)])

        # check if the object has the timestamp attribute TODO ?? really necessary
        # if hasattr(obj, 'logtimestamp'):
        # if it does create a location index
        #  collection.create_index([("datetime", pymongo.GEO2D)])

        try:
            stamp = self.get_clock().now()
            sec_ns = stamp.seconds_nanoseconds()
            fl = float(f"{sec_ns[0]}.{sec_ns[1]}")
            meta["inserted_at"] = datetime.fromtimestamp(fl, timezone.utc)
            # TODO: Retrieving this information seems to be much harder/impossible in ros2
            # meta["inserted_by"] = request._connection_header["callerid"]

            if (
                hasattr(obj, "header")
                and hasattr(obj.header, "stamp")
                and isinstance(obj.header.stamp, Time)
            ):
                stamp = obj.header.stamp
            elif isinstance(obj, TFMessage):
                if obj.transforms:
                    transforms = sorted(
                        obj.transforms, key=lambda m: m.header.stamp, reverse=True
                    )
                    stamp = transforms[0].header.stamp

            sec_ns = stamp.seconds_nanoseconds()
            fl = float(f"{sec_ns[0]}.{sec_ns[1]}")
            meta["published_at"] = datetime.fromtimestamp(fl, timezone.utc)
            meta["timestamp"] = stamp.nanoseconds

            obj_id = dc_util.store_message(collection, obj, meta)
            return MongoInsertMsg.Response(id=str(obj_id))
        except Exception as e:
            import traceback

            print(traceback.format_exc())
            return MongoInsertMsg.Response(id="")

    insert_ros_srv.type = MongoInsertMsg

    def delete_ros_srv(
        self, request: MongoDeleteMsg.Request, response: MongoDeleteMsg.Response
    ) -> MongoDeleteMsg.Response:
        """
        Deletes a message by ID
        """
        # Get the message
        collection = self._mongo_client[request.database][request.collection]
        docs = dc_util.query_message(
            collection, {"_id": ObjectId(request.document_id)}, find_one=True
        )
        if len(docs) != 1:
            return MongoDeleteMsg.Response(success=False)

        message = docs[0]

        # Remove the doc
        collection.remove({"_id": ObjectId(request.document_id)})

        if self.keep_trash:
            # But keep it into "trash"
            bk_collection = self._mongo_client[request.database][
                request.collection + "_Trash"
            ]
            bk_collection.save(message)

        return MongoDeleteMsg.Response(success=True)

    delete_ros_srv.type = MongoDeleteMsg

    def update_ros_srv(
        self, request: MongoUpdateMsg.Request, response: MongoUpdateMsg.Response
    ) -> MongoUpdateMsg.Response:
        """
        Updates a msg in the store
        """
        # rospy.lrosoginfo("called")
        collection = self._mongo_client[request.database][request.collection]

        # build the query doc
        obj_query = self.to_query_dict(request.message_query, request.meta_query)

        # restrict results to have the type asked for
        obj_query["_meta.stored_type"] = request.message.type

        # TODO start using some string constants!

        self.get_logger().debug(f"update spec document: {obj_query}")

        # deserialize data into object
        obj = dc_util.deserialise_message(request.message)

        meta = dc_util.string_pair_list_to_dictionary(request.meta)
        meta["last_updated_at"] = datetime.fromtimestamp(
            self.get_clock().now().seconds_nanoseconds()[0], timezone.utc
        )
        meta["last_updated_by"] = request._connection_header["callerid"]

        (obj_id, altered) = dc_util.update_message(
            collection, obj_query, obj, meta, request.upsert
        )

        return MongoUpdateMsg.Response(id=str(obj_id), success=altered)

    update_ros_srv.type = MongoUpdateMsg

    def to_query_dict(self, message_query, meta_query):
        """
        Decodes and combines the given StringPairList queries into a single mongodb query
        """
        obj_query = dc_util.string_pair_list_to_dictionary(message_query)
        bare_meta_query = dc_util.string_pair_list_to_dictionary(meta_query)
        for k, v in bare_meta_query.items():
            obj_query["_meta." + k] = v
        return obj_query

    def query_messages_ros_srv(
        self, request: MongoQueryMsg.Request, response: MongoQueryMsg.Response
    ) -> MongoQueryMsg.Response:
        """
        Returns t
        """
        collection = self._mongo_client[request.database][request.collection]

        # build the query doc
        obj_query = self.to_query_dict(request.message_query, request.meta_query)

        # restrict results to have the type asked for
        obj_query["_meta.stored_type"] = request.type

        # TODO start using some string constants!

        self.get_logger().debug(f"query document: {obj_query}")

        # this is a list of entries in dict format including meta
        sort_query_dict = dc_util.string_pair_list_to_dictionary(request.sort_query)
        sort_query_tuples = []
        for k, v in sort_query_dict.items():
            try:
                sort_query_tuples.append((k, int(v)))
            except ValueError:
                sort_query_tuples.append((k, v))
            # this is a list of entries in dict format including meta

        projection_query_dict = dc_util.string_pair_list_to_dictionary(
            request.projection_query
        )
        projection_meta_dict = dict()
        projection_meta_dict["_meta"] = 1

        entries = dc_util.query_message(
            collection,
            obj_query,
            sort_query_tuples,
            projection_query_dict,
            request.single,
            request.limit,
        )
        if projection_query_dict:
            meta_entries = dc_util.query_message(
                collection,
                obj_query,
                sort_query_tuples,
                projection_meta_dict,
                request.single,
                request.limit,
            )

        serialised_messages = ()
        metas = ()

        for idx, entry in enumerate(entries):

            # load the class object for this type
            # TODO this should be the same for every item in the list, so could reuse
            cls = rosidl_runtime_py.utilities.get_interface(
                entry["_meta"]["stored_class"]
            )
            # instantiate the ROS message object from the dictionary retrieved from the db
            message = rosidl_runtime_py.set_message_fields(cls(), entry)
            # the serialise this object in order to be sent in a generic form
            serialised_messages = serialised_messages + (
                dc_util.serialise_message(message),
            )
            # add ObjectID into meta as it might be useful later
            if projection_query_dict:
                entry["_meta"]["_id"] = meta_entries[idx]["_id"]
            else:
                entry["_meta"]["_id"] = entry["_id"]
            # serialise meta
            metas = metas + (
                StringPairList(
                    pairs=[
                        StringPair(
                            first=MongoQueryMsg.Request.JSON_QUERY,
                            second=json.dumps(
                                entry["_meta"], default=json_util.default
                            ),
                        )
                    ]
                ),
            )

        return MongoQueryMsg.Response(messages=serialised_messages, metas=metas)

    query_messages_ros_srv.type = MongoQueryMsg

    def query_with_projection_messages_ros_srv(self, req):
        """
        Returns t
        """
        return self.query_messages_ros_srv(req, MongoQueryMsg.Response())

    query_with_projection_messages_ros_srv.type = MongoQuerywithProjectionMsg


def main():
    rclpy.init()
    store = MessageStore()
    rclpy.spin(store, executor=MultiThreadedExecutor())
    store.destroy_node()
    rclpy.shutdown()
