import sys
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from mongodb_store.message_store import MessageStoreProxy
from mongodb_store.util import message_to_namespaced_type

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.node.Node("example_message_store_client")

    msg_store = MessageStoreProxy(node)
    print("Created message store proxy")
    p = Pose(
        position=Point(x=0.0, y=1.0, z=2.0),
        orientation=Quaternion(x=3.0, y=4.0, z=5.0, w=6.0),
    )
    try:
        # insert a pose object with a name, store the id from db
        print("Inserting pose")
        p_id_fav = msg_store.insert_named("my favourite pose", p)

        # you don't need a name (note that this p_id is different than one above)
        p_id_other = msg_store.insert(p)

        # p_id = msg_store.insert(['test1', 'test2'])

        # get it back with a name
        print("querying")
        print(
            f"query result: {msg_store.query_named('my favourite pose', message_to_namespaced_type(Pose))}"
        )

        p.position.x = 666.0

        # update it with a name
        print("Updating named")
        msg_store.update_named("my favourite pose", p)

        p.position.y = 2020.0

        # update the other inserted one using the id
        print("updating with id")
        msg_store.update_id(p_id_other, p)

        stored_p, meta = msg_store.query_id(p_id_other, message_to_namespaced_type(Pose))

        print(stored_p, meta)

        assert stored_p.position.x == 666
        assert stored_p.position.y == 2020
        print("stored object ok")
        print(f"stored object inserted at {meta['inserted_at']} (UTC rostime)")
        print(f"stored object last updated at {meta['last_updated_at']} (UTC rostime)")

        # some other things you can do...

        # get it back with a name
        print("Getting back the object using its name")
        print(
            msg_store.query_named("my favourite pose", message_to_namespaced_type(Pose))
        )

        # try to get it back with an incorrect name, so get None instead
        print("Trying to get back the object with an incorrect name")
        print(
            msg_store.query_named(
                "my favourite position", message_to_namespaced_type(Pose)
            )
        )

        # get all poses
        print("Getting all poses")
        print(msg_store.query(message_to_namespaced_type(Pose)))

        # get the latest one pose
        print("Getting the latest pose")
        print(
            msg_store.query(
                message_to_namespaced_type(Pose),
                sort_query=[("$natural", -1)],
                single=True,
            )
        )

        # get all non-existent typed objects, so get an empty list back
        print("Getting objects with a non-existent type")
        print(msg_store.query("not my type"))

        # get all poses where the y position is 1
        print("All poses where the y position is 1")
        print(msg_store.query(message_to_namespaced_type(Pose), {"message.position.y": 1.0}))

        # get all poses where the y position greater than 0
        print("All poses where the y position is greater than 0")
        print(
            msg_store.query(
                message_to_namespaced_type(Pose), {"message.position.y": {"$gt": 0}}
            )
        )

        print("deleting my favourite pose...")
        msg_store.delete(p_id_fav)
        print("deleting the other pose...")
        msg_store.delete(p_id_other)

    except Exception:
        import traceback

        print(traceback.print_exc())
