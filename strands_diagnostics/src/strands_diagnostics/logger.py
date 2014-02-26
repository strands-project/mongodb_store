# The implementation of the diagnostics logger 
from ros_datacentre.util import *
from strands_diagnostics.msg import DiagnosticMessage
from rosgraph_msgs.msg import Log

import rospy

if check_for_pymongo():
    import pymongo
    have_pymongo = True
else:
    have_pymongo = False
    
import json

class DiagnosticsLogger(object):
    def __init__(self, node):
        self._node =  node
        self.ok = True
        
        # Check for mongodb
        have_db =  wait_for_mongo()
        if not have_db:
            self.ok = False
            return

        # check for pymongo
        if not have_pymongo:
            self.ok = False
            return
        
        self._diag_subscriber =  rospy.Subscriber("/strands_diagnostics",  DiagnosticMessage,  self._diagnostic_cb)

        # Logging of rosout this way is not going to be as efficient as by C++ and mongodb_log
        # package. However, we can split the messages into different collections...
        # TODO: Integrate better with mongodb_log
        self._rosout_subscriber =  rospy.Subscriber("/rosout",  Log, self._rosout_cb)

        host =  rospy.get_param("datacentre_host")
        port =  rospy.get_param("datacentre_port")
        self._mongoclient = pymongo.MongoClient(host, port)
        
        rospy.spin()

        
    def _diagnostic_cb(self, msg):
        collection = self._mongoclient[msg.package][msg.subpackage + "!diagnostics"]
        doc =  {}
        doc["msg"] = json.loads(msg.json_data)
        now =  rospy.get_rostime()
        doc["stamp"] = {"secs": now.secs, "nasecs": now.nsecs,}
        collection.insert(doc)
        
    def _rosout_cb(self, msg):
        if msg.name.find("mongo_server") >  0:
            return
        if msg.name.find("diagnostics_logger") >  0:
            return
        collection = self._mongoclient["rosout_log"][msg.name.replace('/', '')]
        store_message_no_meta(collection, msg)
        
        
