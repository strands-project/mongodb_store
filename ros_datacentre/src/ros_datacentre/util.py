import rospy
import genpy
from std_srvs.srv import Empty
import yaml
import json

"""
Waits for the mongo server, as started through the
ros_datacentre/mongodb_server.py wrapper
Return True on success, False if server not even started.
"""
def wait_for_mongo():
    # Check that mongo is live, create connection
    try:
        rospy.wait_for_service("/datacentre/wait_ready",10)
    except rospy.exceptions.ROSException, e:
        rospy.logerr("Can't connect to MongoDB server. Make sure ros_datacentre/mongodb_server.py node is started.")
        return False
    wait = rospy.ServiceProxy('/datacentre/wait_ready', Empty)
    wait()
    return True

"""
Checks for required version of pymongo, returns True if found
"""
def check_for_pymongo():
    try:
        import pymongo
    except:
        print("ERROR!!!")
        print("Can't import pymongo, this is needed by ros_datacentre.")
        print("Make sure it is installed (sudo pip install pymongo)")
        return False

    if not "MongoClient" in dir(pymongo):
        print ("ERROR!!!")
        print("Can't import required version of pymongo. We need >= 2.3")
        print("Make sure it is installed (sudo pip install pymongo) not apt-get")
        return False
    
    return True

"""
Given a ROS msg and a dictionary of the right values, fill in the msg
"""
def _fill_msg(msg,dic):
    for i in dic:
        if isinstance(dic[i],dict):
            _fill_msg(getattr(msg,i),dic[i])
        else:
            setattr(msg,i,dic[i])
    

"""
Given a document in the database, return metadata and ROS message
"""
def document_to_msg(document, TYPE):
    meta = document["meta"]
    msg = TYPE()
    _fill_msg(msg,document["msg"])
    return meta,msg


""" De-rosify a msg """
def sanitize_value(v):
    if isinstance(v, rospy.Message):
        return msg_to_document(v)
    elif isinstance(v, genpy.rostime.Time):
        return msg_to_document(v)
    elif isinstance(v, genpy.rostime.Duration):
         return msg_to_document(v)
    elif isinstance(v, list):
        return [sanitize_value(t) for t in v]
    else:
        return v

# this version is from mongodb_log but creates non-recreatable entries
# """ Sanitize the input value for addition to the database. Taken from mongodb_log """
# def sanitize_value(v):
#     if isinstance(v, rospy.Message):
#         return message_to_dict(v)
#     elif isinstance(v, genpy.rostime.Time):
#         t = datetime.fromtimestamp(v.secs)
#         return t + timedelta(microseconds=v.nsecs / 1000.)
#     elif isinstance(v, genpy.rostime.Duration):
#         return v.secs + v.nsecs / 1000000000.
#     elif isinstance(v, list):
#         return [sanitize_value(t) for t in v]
#     else:
#         return v

    
"""
Given a ROS message, turn it into something suitable for the datacentre
"""
def msg_to_document(msg):
    d = {}
    for f in msg.__slots__:
        d[f] = sanitize_value(getattr(msg, f))
    return d


"""
Store a ROS message into the DB
"""    
def store_message(collection, msg, meta):
    doc={}
    doc["meta"]=meta
    doc["msg"]=msg_to_document(msg)
    collection.insert(doc)

"""
Store a ROS message sans meta data
"""
def store_message_no_meta(collection, msg):
    doc=msg_to_document(msg)
    collection.insert(doc)

"""
Load a ROS message from the DB, no meta
"""
def document_to_msg_no_meta(document, TYPE):
    msg = TYPE()
    _fill_msg(msg, document)
    return meta, msg
