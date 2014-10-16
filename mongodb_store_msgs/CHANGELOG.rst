^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mongodb_store_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.0.5 (2014-10-09)
------------------

0.0.4 (2014-09-13)
------------------

0.0.3 (2014-08-18)
------------------
* Renamed ros_datacentre to mongodb_store for to fix `#69 <https://github.com/strands-project/ros_datacentre/issues/69>`_.
* Contributors: Nick Hawes

0.0.2 (2014-08-07)
------------------
* Changed collections type to StringList to allow for datacentre comms to task schduler.
* Change action definition to use duration into the past.
* Working and tested dump and restore with time bounds.
* Added dump and restore.
* Adds a service to delete message by ID
* Adds a service to delete message by ID
* Proof of concept working with C++ BSON library.
* Working update method on the python side. Will not work in C++ yet.
* Removed unused MongoQueryID.srv
* Added update message type.
* Message store queries now return meta as well as message.
  This is only in the python client for now, but is simple to add to C++. This could be inefficient, so in the future potentially add non-meta options.
* Now using json.dumps and loads to do better queries from python. C++ is still a pain though.
* Query now returns the messages asked for
* Query structure in place
* Meta stuff working on the way in. Starting to think about querying.
* Added meta information as a list of string pairs
* Working across languages with return value now.
* Working from the C++ end, but this invalidates the Python again.
* Basic idea works python to python
* Service code runs (not working though)
* Adding an insert service and the start of a message store to provide it.
* Contributors: Nick Hawes, cburbridge
