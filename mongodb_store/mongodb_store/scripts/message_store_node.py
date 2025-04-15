"""
Provides a service to store ROS message objects in a mongodb database in JSON.
"""

import json
from datetime import datetime, timezone

import pymongo
from rcl_interfaces.srv import GetParameters
import rclpy
from bson import json_util
from bson.objectid import ObjectId
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
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
        if not param_client.wait_for_service(10):
            raise RuntimeError(
                f"Could not find service {param_client.srv_name} from which to retrieve mongo host and port parameters"
            )

        request = GetParameters.Request()
        request.names = ["mongodb_host", "mongodb_port"]
        resp_future: rclpy.Future = param_client.call_async(request)

        # This is hacky but we need to do it in order to call this service before calling rclpy spin
        rclpy.spin_until_future_complete(
            self,
            resp_future,
            timeout_sec=5,
            executor=SingleThreadedExecutor(),
        )
        # Reset the executor, otherwise the node won't spin properly later
        self.executor = None
        host_value = resp_future.result().values[0]
        port_value = resp_future.result().values[1]
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
                "The option 'replicate_on_write' is now deprecated and will be removed. "
                "Use 'Replication' on MongoDB instead: "
                "https://docs.mongodb.com/manual/replication/"
            )

            extras = self.declare_parameter(
                "mongodb_store_extras",
                [],
                descriptor=ParameterDescriptor(description=""),
            ).value
            self.extra_clients = []
            for extra in extras:
                try:
                    self.extra_clients.append(MongoClient(extra[0], extra[1]))
                except pymongo.errors.ConnectionFailure as e:
                    self.get_logger().warning(
                        f"Could not connect to extra datacentre at {extra[0]}:{extra[1]}"
                    )
            self.get_logger().info(
                f"Replicating content to a futher {len(self.extra_clients)} datacentres"
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
        self.insert_ros_srv(msg)

    def insert_ros_srv(self, req):
        """
        Receives a
        """
        # deserialize data into object
        obj = dc_util.deserialise_message(req.message)
        # convert input tuple to dict
        meta = dc_util.string_pair_list_to_dictionary(req.meta)
        # get requested collection from the db, creating if necessary
        collection = self._mongo_client[req.database][req.collection]
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

        # try:
        stamp = self.get_clock().now().to_msg()
        meta["inserted_at"] = datetime.fromtimestamp(stamp.to_sec(), timezone.UTC)
        meta["inserted_by"] = req._connection_header["callerid"]
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

        meta["published_at"] = datetime.fromtimestamp(stamp.to_sec(), timezone.utc)
        meta["timestamp"] = stamp.to_nsec()

        obj_id = dc_util.store_message(collection, obj, meta)

        if self.replicate_on_write:
            # also do insert to extra datacentres, making sure object ids are consistent
            for extra_client in self.extra_clients:
                extra_collection = extra_client[req.database][req.collection]
                dc_util.store_message(extra_collection, obj, meta, obj_id)

        return str(obj_id)
        # except Exception, e:
        # print e

    insert_ros_srv.type = MongoInsertMsg

    def delete_ros_srv(self, req):
        """
        Deletes a message by ID
        """
        # Get the message
        collection = self._mongo_client[req.database][req.collection]
        docs = dc_util.query_message(
            collection, {"_id": ObjectId(req.document_id)}, find_one=True
        )
        if len(docs) != 1:
            return False

        message = docs[0]

        # Remove the doc
        collection.remove({"_id": ObjectId(req.document_id)})

        if self.keep_trash:
            # But keep it into "trash"
            bk_collection = self._mongo_client[req.database][req.collection + "_Trash"]
            bk_collection.save(message)

            # also repeat in extras
            if self.replicate_on_write:
                for extra_client in self.extra_clients:
                    extra_collection = extra_client[req.database][req.collection]
                    extra_collection.remove({"_id": ObjectId(req.document_id)})
                    extra_bk_collection = extra_client[req.database][
                        req.collection + "_Trash"
                    ]
                    extra_bk_collection.save(message)

        return True

    delete_ros_srv.type = MongoDeleteMsg

    def update_ros_srv(self, req):
        """
        Updates a msg in the store
        """
        # rospy.lrosoginfo("called")
        collection = self._mongo_client[req.database][req.collection]

        # build the query doc
        obj_query = self.to_query_dict(req.message_query, req.meta_query)

        # restrict results to have the type asked for
        obj_query["_meta.stored_type"] = req.message.type

        # TODO start using some string constants!

        self.get_logger().debug(f"update spec document: {obj_query}")

        # deserialize data into object
        obj = dc_util.deserialise_message(req.message)

        meta = dc_util.string_pair_list_to_dictionary(req.meta)
        meta["last_updated_at"] = datetime.fromtimestamp(
            self.get_clock().now().seconds_nanoseconds()[0], timezone.utc
        )
        meta["last_updated_by"] = req._connection_header["callerid"]

        (obj_id, altered) = dc_util.update_message(
            collection, obj_query, obj, meta, req.upsert
        )

        if self.replicate_on_write:
            # also do update to extra datacentres
            for extra_client in self.extra_clients:
                extra_collection = extra_client[req.database][req.collection]
                dc_util.update_message(
                    extra_collection, obj_query, obj, meta, req.upsert
                )

        return str(obj_id), altered

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

    def query_messages_ros_srv(self, req):
        """
        Returns t
        """
        collection = self._mongo_client[req.database][req.collection]

        # build the query doc
        obj_query = self.to_query_dict(req.message_query, req.meta_query)

        # restrict results to have the type asked for
        obj_query["_meta.stored_type"] = req.type

        # TODO start using some string constants!

        self.get_logger().debug(f"query document: {obj_query}")

        # this is a list of entries in dict format including meta
        sort_query_dict = dc_util.string_pair_list_to_dictionary(req.sort_query)
        sort_query_tuples = []
        for k, v in sort_query_dict.items():
            try:
                sort_query_tuples.append((k, int(v)))
            except ValueError:
                sort_query_tuples.append((k, v))
            # this is a list of entries in dict format including meta

        projection_query_dict = dc_util.string_pair_list_to_dictionary(
            req.projection_query
        )
        projection_meta_dict = dict()
        projection_meta_dict["_meta"] = 1

        entries = dc_util.query_message(
            collection,
            obj_query,
            sort_query_tuples,
            projection_query_dict,
            req.single,
            req.limit,
        )
        if projection_query_dict:
            meta_entries = dc_util.query_message(
                collection,
                obj_query,
                sort_query_tuples,
                projection_meta_dict,
                req.single,
                req.limit,
            )

        # keep trying clients until we find an answer
        if self.replicate_on_write:
            for extra_client in self.extra_clients:
                if len(entries) == 0:
                    extra_collection = extra_client[req.database][req.collection]
                    entries = dc_util.query_message(
                        extra_collection,
                        obj_query,
                        sort_query_tuples,
                        projection_query_dict,
                        req.single,
                        req.limit,
                    )
                    if projection_query_dict:
                        meta_entries = dc_util.query_message(
                            extra_collection,
                            obj_query,
                            sort_query_tuples,
                            projection_meta_dict,
                            req.single,
                            req.limit,
                        )
                    if len(entries) > 0:
                        self.get_logger().info("found result in extra datacentre")
                else:
                    break

        serialised_messages = ()
        metas = ()

        for idx, entry in enumerate(entries):

            # load the class object for this type
            # TODO this should be the same for every item in the list, so could reuse
            cls = dc_util.load_class(entry["_meta"]["stored_class"])
            # instantiate the ROS message object from the dictionary retrieved from the db
            message = dc_util.dictionary_to_message(entry, cls)
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

        return [serialised_messages, metas]

    query_messages_ros_srv.type = MongoQueryMsg

    def query_with_projection_messages_ros_srv(self, req):
        """
        Returns t
        """
        return self.query_messages_ros_srv(req)

    query_with_projection_messages_ros_srv.type = MongoQuerywithProjectionMsg


def main():
    rclpy.init()
    store = MessageStore()
    rclpy.spin(store, executor=MultiThreadedExecutor())
    store.destroy_node()
    rclpy.shutdown()
