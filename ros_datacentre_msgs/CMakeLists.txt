cmake_minimum_required(VERSION 2.8.3)
project(ros_datacentre_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_service_files(
  FILES
  MongoInsertMsg.srv
)

generate_messages()

catkin_package(
 CATKIN_DEPENDS message_generation
)

