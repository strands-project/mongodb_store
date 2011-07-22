cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_add_boost_directories()


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
#rosbuild_gensrv()

link_libraries(mongoclient)
rosbuild_add_executable(mongolog_tf src/mongolog_tf.cpp)
rosbuild_add_executable(mongolog_pcl src/mongolog_pcl.cpp)
rosbuild_add_executable(mongolog_cimg src/mongolog_cimg.cpp)
# rosbuild_add_executable(rosmongolog_trimesh src/rosmongolog_trimesh.cpp)
rosbuild_link_boost(mongolog_cimg system)
rosbuild_link_boost(mongolog_pcl system)
rosbuild_link_boost(mongolog_tf system)
#rosbuild_link_boost(mongolog_trimesh system)

