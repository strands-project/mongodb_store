import rospy
import genpy
from std_srvs.srv import Empty
import yaml
import json
import copy
import StringIO
from ros_datacentre_msgs.msg import SerialisedMessage
import importlib

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
Given a document in the database, return metadata and ROS message -- must have been
"""
def document_to_msg_and_meta(document, TYPE):
    meta = document["_meta"]
    msg = TYPE()
    _fill_msg(msg,document["msg"])
    return meta,msg

"""
Given a document return ROS message
"""
def document_to_msg(document, TYPE):
    msg = TYPE()
    _fill_msg(msg,document)
    return meta

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
    doc=msg_to_document(msg)
    doc["_meta"]=meta
    return collection.insert(doc)

# """
# Stores a ROS message into the DB with msg and meta as separate fields
# """    
# def store_message_separate(collection, msg, meta):
#     doc={}
#     doc["_meta"]=meta
#     doc["msg"]=msg_to_document(msg)
#     return collection.insert(doc)



"""
Store a ROS message sans meta data
"""
def store_message_no_meta(collection, msg):
    doc=msg_to_document(msg)
    return collection.insert(doc)


"""
Fill a ROS message from a dictionary, assuming the slots of the message are keys in the dictionary.
"""
def fill_message(message, document):
    for slot in message.__slots__:
        value = document[slot]
        # fill internal structures if value is a dictionary itself
        if isinstance(value, dict):
            fill_message(getattr(message, slot), value)
        else:
            setattr(message, slot, value)    

"""
Create a ROS message from the given dictionary, using fill_message.
"""
def dictionary_to_message(dictionary, cls):
    message = cls()
    fill_message(message, dictionary)
    return message

"""
Peform a query for a stored messages, returning results in list
"""
def query_message(collection, query_doc, find_one):

    if find_one:
        ids = ()
        result = collection.find_one(query_doc)
        if result:
            return [ result ] 
        else:
            return []
    else:
        return [ result for result in collection.find(query_doc) ]


"""
Peform a query for a stored, returning a tuple of id strings
"""
def query_message_ids(collection, query_doc, find_one):

    if find_one:
        ids = ()
        result = collection.find_one(query_doc)
        if result:
            return str(result["_id"]), 
    else:
        return tuple(str(result["_id"]) for result in collection.find(query_doc, {'_id':1}))



def type_to_class_string(type):
    """ 
    Takes a ROS msg type and turns it into a Python module and class name. 
    E.g. from 
    geometry_msgs/Pose 
    to
    geometry_msgs.msg._Pose.Pose
    """    
    parts = type.split('/')
    cls_string = "%s.msg._%s.%s" % (parts[0], parts[1], parts[1])
    return cls_string

def load_class(full_class_string):
    """
    dynamically load a class from a string
    shamelessly ripped from: http://thomassileo.com/blog/2012/12/21/dynamically-load-python-modules-or-classes/
    """
    # todo: cache classes (if this is an overhead)
    class_data = full_class_string.split(".")
    module_path = ".".join(class_data[:-1])
    class_str = class_data[-1]
    module = importlib.import_module(module_path)
    # Finally, we retrieve the Class
    return getattr(module, class_str)


"""
Create a SerialisedMessage instance from a ROS message
"""
def serialise_message(message):
    buf=StringIO.StringIO() 
    message.serialize(buf)
    serialised_msg = SerialisedMessage()
    serialised_msg.msg = buf.getvalue() 
    serialised_msg.type = message._type
    return serialised_msg

"""
Create a ROS message from a SerialisedMessage
"""
def deserialise_message(serialised_message):
    cls_string = type_to_class_string(serialised_message.type)
    cls = load_class(cls_string)
    # instantiate an object from the class
    message = cls()
    # deserialize data into object
    message.deserialize(serialised_message.msg)
    return message
