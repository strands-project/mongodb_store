ros_datacentre
==================

This package wraps up MongoDB database server in ROS, allowing it to be used to store configuration parameters.

Two nodes are provided:
- mongodb_server.py
- config_manager.py

These node depends on MongoDB and the Python client libraries (>=2.3). Install by:

```
sudo apt-get install python-pymongo mongodb
```
If this does not give the required version, you can use:

```
sudo pip install pymongo
```

Running the mongodb_server
--------------------------
The start the datacentre:

```
rosparam set datacentre_port 62345
rosparam set datacentre_host bob # note that if using multiple machines, 'localhost' is no good

rosrun ros_datacentre mongodb_server.py
```

By default, the mongod database will be stored in `/opt/strands/ros_datacentre`. This can be overridden by setting the private parameter ~database_path for the node. If it is the first time that the database is used, be sure to first run

```mkdir  /opt/strands/ros_datacentre``` 

If you prefer to use different mongodb instance, set the datacentre_* parameters accordingly.



Config Manager Overview
-----------------------

The config manager provides a centralised way to store robot application parameters, with optional site-specific overrides. All configurations are stored inside the ros_datacentre mongodb, within a database named "configs". 

Two levels of parameters are considered:

1) Global default parameters. 
These should be "working defaults" - so all essential parameters at least have a default value. For example, if a robot application requires some calibration data, default values should be provided.
These parameters are shared among sites, so are stored inside github under ros_datacentre/defaults. When the config manager is started, all .yaml files stored in this folder will be examined. Any new default parameters will be inserted into the "defaults" collection within the configs database.

2) Local parameters.
These parameters override the same named  global default parameters, allowing site-specific parameter setting. They are stored within the database inside the "local" collection.

At start up, the config manager places all parameters onto the ros parameter server to allow interoperability with existing software. Parameters can also be queried using the /config_manager/get_param service, or by directly connection to and querying the mongo database server.

Likewise, local parameter overrides can be set using the /config_manager/set_param service or by directly editing the "local" collection in the configs database.


Running config manager
----------------------

To start the config manager, make sure that you have the mongo db running then:


```
rosrun ros_datacentre config_manager.py
```

This will load all parameters onto the ros parameter server, which can be checked with:
```
rosparam list
```




Reading parameters
------------------

There are three methods to access parameter values:
1) Use the copy on the ros parameter server:
```
rosparam get /my/parameter
```
and likewise with the rospy and roscpp libraries.
2) Use the config_manager service:
```
rosservice call /config_manager/get_param "param_name: '/my/parameter'" 
```
3) Using the database server directly


Setting parameters
------------------
Default parameters are set by placing them in the yaml files in the defaults directory in ros_datacentre. This way, default parameters are added to the github repo and shared between all users.

The local parameter overrides can be set in 2 ways:
1) Using the config_manager service:
```
rosservice call /config_manager/set_param "param: '{\"path\":\"/chris\",\"value\":43}'" 
```

Note the syntax of the parameter: it is a json representation of a dictionary with path and value keys.

2) Using the config_manager service:
```
rosservice call /config_manager/save_param name_of_the_parameter_to_be_saved
```
Note: This will save the current value of the parameter into the locals database

3) Using the database server directly


Launch files
============
Both mongodb and config_manager can be started together using the datacentre.launch file:

```
HOSTNAME=yourhost roslaunch ros_datacentre datacentre.launch db_path:=/path/to/db db_port:=62345
```

The HOSTNAME env variable is required; db_path will default to /opt/strands/ros_datacentre and db_port will default to 62345. 


Replication
===========

If the constructor arcgument to the message store node `replicate_on_write` is set to true, replication of the message store parts of the datacentre is done manually to allow different content to appear on different hosts. A list of hosts and ports where replications should be made can be set via the `ros_datacentre_extras` parameter:

```yaml
ros_datacentre_extras: [["localhost", 62344], ["localhost", 62333]]
```

Inserts and updates are performed acorss the main and replicant datacentres.

If `ros_datacentre_extras` is set (regardless of `replicate_on_write`), queries are performed on the main first, and if nothing found, the replicants are tried.

You can launch additional datacentres as follows, e.g.

```bash
rosrun ros_datacentre mongodb_server.py _master:=false _database_path:=/opt/strands/strands_datacentre_62344 _host:=localhost _port:=62344
rosrun ros_datacentre mongodb_server.py _master:=false _database_path:=/opt/strands/strands_datacentre_62333 _host:=localhost _port:=62333
```

You can test if this works by adding some things to the message store, deleting them from the master using RoboMongo (not the message store as the deletes are replicated), then running queries.

Action Server for Replication
-----------------------------

The `MoveEntries` action and the corresponding action server:

```bash
rosrun ros_datacentre replicator_node.py 
```

(which is included in `datacentre.launch`)

allows you to bulk copy or move entries from message store collections to the mongod instances defined under `ros_datacentre_extras`. The client accepts a list of collection names and uses the `meta["inserted_at"]` field of the message store entries to replicate or move all entries that were inserted before a particular time. If no time is provided then the default is 24 hours ago. There is an example client that does this for a list of collections specified on the command line. This *moves* entries inserted 24 hours ago or earlier.

```bash
rosrun ros_datacentre replicator_client.py message_store robblog scheduling_problems
```

**NOTE THAT this all makes `update` operations a bit uncertain, so please do not use this type of replication on collections you plan to use update on.**

