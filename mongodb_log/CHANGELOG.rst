^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mongodb_log
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Renamed rosparams `datacentre_` to `mongodb_`.
  Fixes `#69 <https://github.com/strands-project/ros_datacentre/issues/69>`_
* Renamed ros_datacentre to mongodb_store for to fix `#69 <https://github.com/strands-project/ros_datacentre/issues/69>`_.
* Contributors: Nick Hawes

0.0.2 (2014-08-07)
------------------
* Dynamically choose MongoDB API
  Use Connection if using an older mongopy, otherwise use MongoClient.
* Cleaned up boilerplate in mongodb_log package.xml
  Removed a bunch of XML comments (that came from the package.xml
  template) from the package.xml file. Added pymongo as a run dependency.
* Main process no longer calls init_node.
  This fixes bugs related to calling init_node multiple times in the same
  process. Main process now has its own signal handler for shutting down
  cleanly.
* Added 'inserted_at' meta with proper date object to logged data.
  This added compatibility with message store and also allows native date queries on results.
* Changes to how meta info is stored.
* Added boost filesystem for new version of ld.
* Added mongo dependency
* More benchmark removal
* REmoved rrd bits.
* Removing benchmarking stuff.
* Restructuring for new repo position.
* Moved all files into mongodb_log subdirectory for later inclusion in a broader package.
* Contributors: Alex Bencz, Christian Dondrup, Nick Hawes
