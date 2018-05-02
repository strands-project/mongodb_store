^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libmongocxx_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.8 (2018-05-02)
------------------

0.3.7 (2017-12-14)
------------------
* Merge pull request `#210 <https://github.com/strands-project/mongodb_store/issues/210>`_ from hawesie/kinetic-devel
  Updated to build on OS X with the mongo client legacy version.
* Updated to build on OS X with the mongo client legacy version.
  This makes a couple of changes. For libmongocxx_ros the paths for openssl are added to the scons command and all ".so" and set to ".so" or ".dylib" depending on platform. For mongodb_log this removes the explicit linking of mongoclient which isn't needed since catkin_libraries includes the lib created by libmongocxx_ros.
* Merge pull request `#207 <https://github.com/strands-project/mongodb_store/issues/207>`_ from furushchev/fix-mongocxx
  Use system mongocxx client if possible
* libmongocxx_ros: try to use system mongocxx lib if possible
* Contributors: Nick Hawes, Yuki Furuta

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
* had to add `ca-certificates` as `build_depend`
  as the build step (git clone) in the cmake command to build the binary packages failed
* Contributors: Marc Hanheide

0.3.1 (2017-08-24)
------------------

0.2.2 (2017-06-28)
------------------
* fixed wrong install place
* Contributors: Marc Hanheide

0.2.1 (2017-06-28)
------------------
* added git which was missing for the build
* Contributors: Marc Hanheide

0.2.0 (2017-06-28)
------------------
* attempt to fix the cmake hell
* fixed install problem
* some attempts to build libmongocxx internally
* Contributors: Marc Hanheide
