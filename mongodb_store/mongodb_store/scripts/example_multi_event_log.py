from datetime import *

import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool

from mongodb_store.message_store import MessageStoreProxy
from mongodb_store_msgs.msg import StringPairList, StringPair
from mongodb_store.util import message_to_namespaced_type

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.node.Node("example_multi_event_log")

    try:

        # let's say we have a couple of things that we need to store together
        # these could be some sensor data, results of processing etc.
        pose = Pose(
            position=Point(x=0.0, y=1.0, z=2.0),
            orientation=Quaternion(x=3.0, y=4.0, z=5.0, w=6.0),
        )
        point = Point(x=7.0, y=8.0, z=9.0)
        quaternion = Quaternion(x=10.0, y=11.0, z=12.0, w=13.0)
        # note that everything that is pass to the message_store must be a ros message type
        # therefore use std_msg types for standard data types like float, int, bool, string etc
        result = Bool(data=True)

        # we will store our results in a separate collection
        msg_store = MessageStoreProxy(node, collection="pose_results")

        messages_to_store = [pose, point, quaternion, result]
        spl = StringPairList()
        for message in messages_to_store:
            # Each pair in the string pair list will be the type of message stored, and the id of the relevant
            # message in the collection
            spl.pairs.append(
                StringPair(
                    first=message_to_namespaced_type(message),
                    second=msg_store.insert(message),
                )
            )

        # and add some meta information
        meta = {"description": "this wasn't great"}
        sec_ns = node.get_clock().now().seconds_nanoseconds()
        fl = float(f"{sec_ns[0]}.{sec_ns[1]}")
        meta["result_time"] = datetime.fromtimestamp(fl, timezone.utc)
        msg_store.insert(spl, meta=meta)

        # now let's get all our logged data back
        results = msg_store.query(message_to_namespaced_type(StringPairList))
        for message, meta in results:
            if "description" in meta:
                print(f"description: {meta['description']}")
            print(f"result time (UTC from rostime): {meta['result_time']}")
            print(f"inserted at (UTC from rostime): {meta['inserted_at']}")
            pose = msg_store.query_id(
                message.pairs[0].second, message_to_namespaced_type(Pose)
            )[0]
            point = msg_store.query_id(
                message.pairs[1].second, message_to_namespaced_type(Point)
            )[0]
            quaternion = msg_store.query_id(
                message.pairs[2].second, message_to_namespaced_type(Quaternion)
            )[0]
            result = msg_store.query_id(
                message.pairs[3].second, message_to_namespaced_type(Bool)
            )[0]
            print(pose)
            print(point)
            print(quaternion)
            print(result)

    except Exception:
        import traceback

        print(traceback.print_exc())
