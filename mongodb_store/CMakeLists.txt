cmake_minimum_required(VERSION 2.8.3)
project(mongodb_store)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation rospy std_msgs std_srvs)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################


## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetParam.srv
  SetParam.srv
  MongoFind.srv
  MongoUpdate.srv
  MongoInsert.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES mongodb_store
#  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/mongodb_store/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
endif()


if (CATKIN_ENABLE_TESTING)
  add_rostest(tests/message_store.test)
endif()