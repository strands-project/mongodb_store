cmake_minimum_required(VERSION 2.8.3)
project(mongodb_store_cpp_client)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mongodb_store_msgs
  # pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread program_options filesystem)

find_library(MONGOCLIENT_LIB mongoclient)
find_library(SSL_LIB ssl)
find_library(CRYPTO_LIB cryto)

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
 LIBRARIES message_store ${MONGOCLIENT_LIB} ${SSL_LIB} ${CRYPTO_LIB}
 CATKIN_DEPENDS mongodb_store_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(mongodb_store_cpp_client
#   src/${PROJECT_NAME}/mongodb_store_cpp_client.cpp
# )



add_definitions(-std=c++0x)

add_library(message_store src/message_store.cpp )

add_executable(example_mongodb_store_cpp_client src/example_mongodb_store_cpp_client.cpp)
add_executable(example_multi_event_log src/example_multi_event_log.cpp)
# add_executable(pc_test src/point_cloud_test.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

add_dependencies(example_mongodb_store_cpp_client mongodb_store_msgs_generate_messages_cpp )
add_dependencies(message_store mongodb_store_msgs_generate_messages_cpp )

target_link_libraries(message_store
  ${MONGOCLIENT_LIB}
  ${SSL_LIB}
  ${CRYPTO_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)



# Specify libraries to link a library or executable target against
target_link_libraries(example_mongodb_store_cpp_client
  message_store
  ${MONGOCLIENT_LIB}
  ${SSL_LIB}
  ${CRYPTO_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(example_multi_event_log
  message_store
  ${MONGOCLIENT_LIB}
  ${SSL_LIB}
  ${CRYPTO_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(example_multi_event_log
  message_store
  ${MONGOCLIENT_LIB}
  ${SSL_LIB}
  ${CRYPTO_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS example_mongodb_store_cpp_client example_multi_event_log message_store #pc_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/mongodb_store/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mongodb_store.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(unittests/test_messagestore.py)




if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)

  add_executable(message_store_cpp_test tests/message_store_cpp_test.cpp)

  target_link_libraries(message_store_cpp_test
  message_store 
  ${MONGOCLIENT_LIB} ${SSL_LIB} ${CRYPTO_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtest
)

  add_rostest(tests/message_store_cpp_client.test)
endif()
