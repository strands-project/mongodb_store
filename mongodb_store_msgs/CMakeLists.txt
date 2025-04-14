cmake_minimum_required(VERSION 3.10)
project(mongodb_store_msgs)

if(POLICY CMP0148)
# don't show warnings about this policy
cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake)
find_package(std_msgs)
find_package(action_msgs)
find_package(builtin_interfaces)
find_package(rosidl_default_generators)

file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/msg/*.msg"
    "${CMAKE_CURRENT_LIST_DIR}/srv/*.srv"
    "${CMAKE_CURRENT_LIST_DIR}/action/*.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()