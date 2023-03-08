# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8)
project(pgm_map_creator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
# Undefined reference to symbol 'pthread_create@@GLIBC_2.2.5'
# https://stackoverflow.com/questions/34143265/undefined-reference-to-symbol-pthread-createglibc-2-2-5
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

#[[include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  #pkg_check_modules(GAZEBO gazebo)
  if($ENV{ROS_DISTRO} STREQUAL "kinetic" OR $ENV{ROS_DISTRO} STREQUAL "lunar")
    pkg_check_modules(SDF REQUIRED sdformat4)
  elseif($ENV{ROS_DISTRO} STREQUAL "melodic")
    pkg_check_modules(SDF REQUIRED sdformat6)
  elseif($ENV{ROS_DISTRO} STREQUAL "noetic")
    pkg_check_modules(SDF REQUIRED sdformat9)
  else()
    pkg_check_modules(SDF REQUIRED sdformat)
  endif()
endif()#]]

# http://sdformat.org/tutorials?tut=quickstart
# find_package(sdformat<version of sdformat> REQUIRED) # sdformat7 or higher
if($ENV{ROS_DISTRO} STREQUAL "noetic")
  find_package(SDFormat9 REQUIRED)
else()
  find_package(SDFormat REQUIRED)
endif()

find_package(Gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
find_package(Boost REQUIRED system COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pgm_map_creator
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  
  ${GAZEBO_PROTO_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_executable (request_publisher src/request_publisher.cc)
target_link_libraries(request_publisher collision_map_creator_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${GAZEBO_PROTO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(request_publisher collision_map_creator_msgs)

add_library(collision_map_creator SHARED src/collision_map_creator.cc )
target_link_libraries(collision_map_creator collision_map_creator_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${GAZEBO_PROTO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(collision_map_creator collision_map_creator_msgs)
