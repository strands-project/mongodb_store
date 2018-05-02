^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mongodb_store_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.8 (2018-05-02)
------------------
* Merge pull request `#220 <https://github.com/strands-project/mongodb_store/issues/220>`_ from furushchev/replication-with-query
  Replication with query
* mongodb_store_msgs: add query slot to MoveEntries action
* Merge remote-tracking branch 'origin/kinetic-devel' into fix/encode-utf8
* Merge remote-tracking branch 'origin/kinetic-devel' into option-query-extra
  Conflicts:
  mongodb_store/scripts/message_store_node.py
* Merge pull request `#215 <https://github.com/strands-project/mongodb_store/issues/215>`_ from furushchev/remove-dup-projection
  mongodb_store: Remove duplicated codes for projection
* mongodb_store: remove duplicated codes for querying with projection
* Contributors: Furushchev, Nick Hawes

0.3.7 (2017-12-14)
------------------


0.3.6 (2017-09-14)
------------------

0.3.5 (2017-09-03)
------------------

0.3.4 (2017-09-03)
------------------

0.3.3 (2017-09-02)
------------------

0.3.2 (2017-08-31)
------------------

0.3.1 (2017-08-24)
------------------


0.3.0 (2017-08-24)
------------------

0.2.2 (2017-06-28)
------------------

0.2.1 (2017-06-28)
------------------

0.2.0 (2017-06-28)
------------------
* indigo-0.1.30
* updated Changelogs
* indigo-0.1.29
* updated Changelogs
* [mongodb_store_msgs] add Insert.msg
* indigo-0.1.28
* updated Changelogs
* indigo-0.1.27
* updated Changelogs
* Contributors: Nick Hawes, Yuki Furuta

0.1.30 (2017-06-23)
-------------------

0.1.29 (2017-06-19)
-------------------
* [mongodb_store_msgs] add Insert.msg
* Contributors: Nick Hawes, Yuki Furuta

0.1.28 (2016-11-09)
-------------------

0.1.27 (2016-11-01)
-------------------

0.1.26 (2016-10-14)
-------------------
* Created a new service for querying with projections
* Projection field to the queries is added
* indigo-0.1.25
* Updating changelogs
* indigo-0.1.24
* updated Changelogs
* Contributors: Nick Hawes, hkaraoguz

0.1.25 (2016-04-28)
-------------------

0.1.24 (2016-04-19)
-------------------

0.1.23 (2016-04-19)
-------------------


0.1.22 (2016-02-23)
-------------------
* indigo-0.1.22
* updated Changelogs
* Contributors: Nick Hawes


0.1.20 (2015-11-11)
-------------------


0.1.19 (2015-10-28)
-------------------


0.1.18 (2015-10-28)
-------------------
* updated Changelogs
* [mongodb_store] add limit argument for query
* Contributors: Nick Hawes, Yuki Furuta

0.1.17 (2015-09-01)
-------------------

0.1.16 (2015-08-04)
-------------------

0.1.15 (2015-05-10)
-------------------

0.1.14 (2015-04-27)
-------------------

0.1.13 (2015-04-22)
-------------------

0.1.12 (2015-02-09)
-------------------

0.1.11 (2015-02-09)
-------------------

0.1.10 (2014-11-23)
-------------------
* Replication now has db configurable.
  This fixes `#54 <https://github.com/strands-project/mongodb_store/issues/54>`_.
* Contributors: Nick Hawes

0.1.9 (2014-11-18)
------------------

0.1.8 (2014-11-11)
------------------

0.1.7 (2014-11-09)
------------------

0.1.6 (2014-11-06)
------------------

0.1.5 (2014-11-05)
------------------

0.1.4 (2014-10-29)
------------------
* add sort option on query
* Contributors: Furushchev

0.1.3 (2014-10-21)
------------------

0.1.2 (2014-10-20)
------------------

0.1.1 (2014-10-17)
------------------

0.1.0 (2014-10-16)
------------------

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
