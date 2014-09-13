^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mongodb_store
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* added mongod
* Add son_manipulator import
* Added test mode to mongodb_server.py
  This generates a random port to listen on and creates a corresponding dbpath under /tmp. The port is tested to see if it's free before it's used.
  This closes `#77 <https://github.com/strands-project/mongodb_store/issues/77>`_ and `#75 <https://github.com/strands-project/mongodb_store/issues/75>`_.
* Contributors: Chris Burbridge, Marc Hanheide, Nick Hawes

0.0.3 (2014-08-18)
------------------
* Renamed rosparams `datacentre_` to `mongodb_`.
  Fixes `#69 <https://github.com/strands-project/ros_datacentre/issues/69>`_
* More renaming to mongodb_store
* Renamed launch file.
* Renamed ros_datacentre to mongodb_store for to fix `#69 <https://github.com/strands-project/ros_datacentre/issues/69>`_.
* Contributors: Nick Hawes

0.0.2 (2014-08-07)
------------------
* Fix `#65 <https://github.com/strands-project/mongodb_store/issues/65>`_. Check entry exists before accessing value.
* Dynamically choose MongoDB API
  Use Connection if using an older mongopy, otherwise use MongoClient.
* Remove dependency on bson > 2.3
  Use old hook/default interface to avoid having to install bson 2.3 from
  pip.
* Remove dependency on pymongo > 2.3
  In older versions of pymongo, Connection serves the same purpose as
  MongoClient. Updated scripts to use Connection instead of MongoClient.
  This allows the package to work with the existing rosdep definitions for
  python-pymongo (shich use the .deb version).
* Fix `#63 <https://github.com/strands-project/mongodb_store/issues/63>`_. Update pass through son manipulators.
* Fix `#60 <https://github.com/strands-project/mongodb_store/issues/60>`_. Add SONManipulator for xmlrpclib binary data.
* datacentre documentation for python
* docstrings in util module
* message store docstrings
* ready for update to use google docstrings
* adding processing of source documentation
* sphinx configuration and index
* sphinx framework for documentation
* Waiting for datacentre.
* Merge pull request `#49 <https://github.com/strands-project/mongodb_store/issues/49>`_ from hawesie/hydro-devel
  Added replication for message store
* Changed collections type to StringList to allow for datacentre comms to task schduler.
* Change action definition to use duration into the past.
* Switched default time to 24 hours ago rather than now, to allow easier use in scheduler.
* Finishing off replicator node.
  * Added to launch file
  * Added to README
  * made client time 24 hours
* Added some minor sanity checks.
* Working and tested dump and restore with time bounds.
* Added dump and restore.
* Fixed empty list error.
* Adding Machine tag to datacentre.launch
* Tested replication and it passes first attempts.
* Adding first pass stuff for replication.
* Deletion now actually deletes...
* Added cpp example of logging multiple messages together.
* Added example of way to log related events to message store.
* Added examples of id-based operations.
* Added update_id method for updating stored object using ObjectID.
* Added updated time and caller too.
* Added inserter id and time to meta.
* Made wait more obvious
* Working binary with pointclouds.
* Added cpp example of logging multiple messages together.
* Added example of way to log related events to message store.
* Added examples of id-based operations.
* Added update_id method for updating stored object using ObjectID.
* Added updated time and caller too.
* Added inserter id and time to meta.
* Made wait more obvious
* Working binary with pointclouds.
* Fixed problem with unicode strings in headers.
* updated pkg name ros_mongodb_datacentre to mongodb_store
* Adding delete function to MessageStoreProxy and using it in unittest.
* Adds a service to delete message by ID
* Adding an initial rostest for the message store proxy.
* Returning id correctly from service call.
* Made id query return single element.
* Added ObjectID into meta on query return
* Now returning from query srv
* Added ability to query message store by ObjectId (python only for now).
  Also added some little bits of logging.
* Adding delete function to MessageStoreProxy and using it in unittest.
* Adds a service to delete message by ID
* Adding an initial rostest for the message store proxy.
* Merge pull request `#28 <https://github.com/strands-project/mongodb_store/issues/28>`_ from hawesie/hydro-devel
  Changes for strands_executive
* Returning id correctly from service call.
* Made id query return single element.
* [message_store] fixing query service
* Added ObjectID into meta on query return
* Now returning from query srv
* Added ability to query message store by ObjectId (python only for now).
  Also added some little bits of logging.
* [message-store] Dealing with lists in stored messages. Bug `#25 <https://github.com/strands-project/mongodb_store/issues/25>`_
* fixed update bug where meta info not updated got dropped from the db
* Made sure name is set correctly with using update named.
* All C++ message_store using BSON and meta returns are in json.
  This means that any legal JSON can now be used for a meta description of an object.
* Proof of concept working with C++ BSON library.
* Adding C++ interface for update.
  Fixed compile issues for srv api change.
* Working update method on the python side. Will not work in C++ yet.
* Message store queries now return meta as well as message.
  This is only in the python client for now, but is simple to add to C++. This could be inefficient, so in the future potentially add non-meta options.
* Moved default datacentre path back to /opt/strands
* Switched strands_datacentre to mongodb_store in here.
* Set default database and collection to be message_store.
  We decided to set these in some way I can't quite recall...
* Added message store to launch file.
  Also made HOSTNAME optional.
* C++ queries are working in a basic form.
* C++ query works
* Now using json.dumps and loads to do better queries from python. C++ is still a pain though.
* Query now returns the messages asked for
* Query structure in place
* Meta stuff working on the way in. Starting to think about querying.
* Added meta in agreed format.
* Wrapping python functionality into a class.
* Working across languages with return value now.
* Works in both languages now!
* Working from the C++ end, but this invalidates the Python again.
* Basic insert chain will work in python. Now on to C++.
* Basic idea works python to python
* Service code runs (not working though)
* Adding an insert service and the start of a message store to provide it.
* Changed db path to be more general.
* Updated launch file.
* Moved strands_datacentre to mongodb_store
* Contributors: Alex Bencz, Bruno Lacerda, Chris Burbridge, Jaime Pulido Fentanes, Nick Hawes, Thomas Fäulhammer, cburbridge
