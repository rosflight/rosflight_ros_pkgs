cmake_minimum_required(VERSION 2.8.3)
project(fcu_common)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_message_files(
  FILES
  GPS.msg
  OutputRaw.msg
  RCRaw.msg
  Command.msg
  Barometer.msg
  Airspeed.msg
  Attitude.msg
  State.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fcu_common
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs
)

add_definitions(-std=c++11)

add_library(fcu_common
  src/simple_pid.cpp
  include/fcu_common/simple_pid.h
)
target_link_libraries(fcu_common ${catkin_LIBRARIES})

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(joy src/joy.cpp)
add_dependencies(joy fcu_common_generate_messages_cpp)
target_link_libraries(joy ${catkin_LIBRARIES})
