cmake_minimum_required(VERSION 2.8.3)
project(libmongocxx_ros)

find_package(catkin REQUIRED)

find_package(OpenSSL REQUIRED)
# workaround for exposing OpenSSL libraries
set(OpenSSL_LIBRARIES ${OPENSSL_LIBRARIES})
set(OpenSSL_INCLUDE_DIRS ${OPENSSL_INCLUDE_DIR})

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(LIB_EXT dylib)
else()
    set(LIB_EXT so)
endif()

#for ROS indigo compile mongocxx without c++11 support
if(DEFINED ENV{ROS_DISTRO})
  if(NOT $ENV{ROS_DISTRO} STREQUAL "indigo")
    set(SCONS_CXX11_FLAG "--c++11=on")
    message(STATUS "Building with C++11 support")
  else() 
    message(STATUS "ROS Indigo: building without C++11 support")
  endif()
else()
  message(STATUS "Environmental variable ROS_DISTRO not defined, checking OS version")
  execute_process(COMMAND lsb_release -cs 
    OUTPUT_VARIABLE RELEASE_CODENAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT ${RELEASE_CODENAME} STREQUAL "trusty")	
    set(SCONS_CXX11_FLAG "--c++11=on")
    message(STATUS "Building with C++11 support")
  else()
    message(STATUS "Building without C++11 support")
  endif()
endif()

# find MongoClient from system first
# if not found, downloads and compiles from sources
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
find_package(MongoClient)
if (MongoClient_FOUND)
  message(STATUS "Found System MongoClient. Copying them into ROS workspace")
  add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/include
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CATKIN_DEVEL_PREFIX}/include)
  add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/include/mongo
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${MongoClient_INCLUDE_DIR}/mongo ${CATKIN_DEVEL_PREFIX}/include/mongo
    DEPENDS ${CATKIN_DEVEL_PREFIX}/include)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/include/mongo
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CATKIN_DEVEL_PREFIX}/include/mongo ${PROJECT_SOURCE_DIR}/include/mongo
    DEPENDS ${CATKIN_DEVEL_PREFIX}/include/mongo)
  add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/lib
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CATKIN_DEVEL_PREFIX}/lib)
  add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/lib/libmongoclient.${LIB_EXT}
    COMMAND ${CMAKE_COMMAND} -E copy ${MongoClient_LIBRARY} ${CATKIN_DEVEL_PREFIX}/lib/libmongoclient.${LIB_EXT}
    DEPENDS ${CATKIN_DEVEL_PREFIX}/lib)
  add_custom_target(mongocxx
    DEPENDS
    ${CATKIN_DEVEL_PREFIX}/lib/libmongoclient.${LIB_EXT}
    ${PROJECT_SOURCE_DIR}/include/mongo
    ${CATKIN_DEVEL_PREFIX}/include/mongo)
else()
  message(STATUS "Not Found System MongoClient. Downloading...")
  include(ProcessorCount)
  processorcount(PROCESSOR_COUNT)
  include(ExternalProject)
  externalproject_add(mongocxx
    GIT_REPOSITORY https://github.com/mongodb/mongo-cxx-driver.git
    GIT_TAG releases/legacy
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}/mongocxx"
    BUILD_IN_SOURCE 1
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND scons -j ${PROCESSOR_COUNT} --ssl --sharedclient --disable-warnings-as-errors --prefix=${CATKIN_DEVEL_PREFIX} --cpppath=${OPENSSL_INCLUDE_DIR} --libpath=${OPENSSL_ROOT_DIR}/lib ${SCONS_CXX11_FLAG} install && mkdir -p ${CMAKE_CURRENT_SOURCE_DIR}/include && ${CMAKE_COMMAND} -E copy_directory ${CATKIN_DEVEL_PREFIX}/include ${CMAKE_CURRENT_SOURCE_DIR}/include
    INSTALL_COMMAND ""
  )
endif()

# dependencies of MongoClient
find_package(Boost REQUIRED COMPONENTS system thread program_options filesystem)


catkin_package(
  LIBRARIES mongoclient
  INCLUDE_DIRS include
  DEPENDS Boost OpenSSL)

add_library(mongoclient_imp SHARED IMPORTED)
set_target_properties(mongoclient_imp PROPERTIES IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/lib/libmongoclient.${LIB_EXT})

add_library(mongoclient)
add_custom_command(TARGET mongoclient
  POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CATKIN_DEVEL_PREFIX}/lib/libmongoclient.${LIB_EXT} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/)
set_target_properties(mongoclient PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(mongoclient_imp mongocxx)
add_dependencies(mongoclient mongoclient_imp)

# Mark executables and/or libraries for installation
install(TARGETS mongoclient
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark cpp header files for installation
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
