cmake_minimum_required(VERSION 3.10)
project(pgm_map_creator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++17")
message(STATUS "${GAZEBO_CXX_FLAGS}")
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclpy
  nav_msgs
  tf2
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_library(collision_map_creator SHARED src/collision_map_creator.cc)
ament_target_dependencies(collision_map_creator ${DEPENDENCIES})
target_link_libraries(collision_map_creator 
  collision_map_creator_msgs 
  ${Boost_LIBRARIES} 
  ${GAZEBO_LIBRARIES} 
  ${SDF_LIBRARIES}
)

add_executable (request_publisher src/request_publisher.cc)
ament_target_dependencies(request_publisher ${DEPENDENCIES})
target_link_libraries(request_publisher 
  collision_map_creator_msgs 
  ${Boost_LIBRARIES} 
  ${GAZEBO_LIBRARIES} 
  ${SDF_LIBRARIES}
)

install(TARGETS request_publisher collision_map_creator
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY launch msgs maps world DESTINATION share/${PROJECT_NAME}/)

ament_package()