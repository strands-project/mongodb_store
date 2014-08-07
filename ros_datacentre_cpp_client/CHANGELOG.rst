^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_datacentre_cpp_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2014-08-07)
------------------
* Fixed complilation under Ubuntu.
  * removed use of toTimeT()
  * add_definitions(-std=c++0x) to allow new C++ features
* Deletion now actually deletes...
* Exporting message_store library from package.
* Merge branch 'hydro-devel' of https://github.com/hawesie/ros_datacentre into hydro-devel
  Conflicts:
  ros_datacentre_cpp_client/CMakeLists.txt
  ros_datacentre_cpp_client/include/ros_datacentre/message_store.h
* Cleaned up differences between two commits.
* Added updateID to cpp client.
* Added cpp example of logging multiple messages together.
* Changed order of MessageStoreProxy constructor arguments.
  This was done to allow more natural changing of parameters in a sensible order. It's more likely you want to change collection name first, so that is the first parameter, leaving remainder as default.
* Added point cloud test, but not including in compilation.
* Working binary with pointclouds.
* Added updateID to cpp client.
* Added cpp example of logging multiple messages together.
* Changed order of MessageStoreProxy constructor arguments.
  This was done to allow more natural changing of parameters in a sensible order. It's more likely you want to change collection name first, so that is the first parameter, leaving remainder as default.
* Added point cloud test, but not including in compilation.
* Working binary with pointclouds.
* Renamed the library to message_store and moved some files around
* Added a ros_datacentre_cpp_client library to avoid multiple definitions of some symbols
* Fixed multiple definition error in C++
* Added rostest launch file.
* Renamed to match convention.
* Added test class for cpp interface.
* Query methods now only return true when something was found.
* Added delete to example script.
* Added constant for empty bson obj.
* Added queryID to C++ side. Insert operations now return IDs. This closes `#29 <https://github.com/strands-project/ros_datacentre/issues/29>`_.
  Minor formatting.
* Changed to get the deserialisation length from the vector length.
  This removes the bug where variable length types were incorrectly deserialised. Thanks to @nilsbore for reporting the bug.
* Changed to get the deserialisation length from the vector length.
  This removes the bug where variable length types were incorrectly deserialised. Thanks to @nilsbore for reporting the bug.
* swapping order of target link libraries to fix compiling error
* Made sure name is set correctly with using update named.
* Changed order of parameters for updateNamed to allow people to ignore BSON for as long as possible.
* All C++ message_store using BSON and meta returns are in json.
  This means that any legal JSON can now be used for a meta description of an object.
* Proof of concept working with C++ BSON library.
* Added and tested update interface to C++ side.
* Adding C++ interface for update.
  Fixed compile issues for srv api change.
* Set default database and collection to be message_store.
  We decided to set these in some way I can't quite recall...
* C++ queries are working in a basic form.
* C++ query works
* Query now returns the messages asked for
* Meta stuff working on the way in. Starting to think about querying.
* Renamed file to match python side
* Default values provided
* Moving some functionality to header file for client utils.
* Working from the C++ end, but this invalidates the Python again.
* Contributors: Bruno Lacerda, Nick Hawes, Rares Ambrus
