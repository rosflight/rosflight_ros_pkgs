cmake_minimum_required(VERSION 2.8.3)
project(rosflight_common)

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
  Status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosflight_common
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs
)

add_definitions(-std=c++11)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(rosflight_common
  src/simple_pid.cpp
  src/turbomath.cpp
)
add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosflight_common ${catkin_LIBRARIES})

add_executable(joy src/joy.cpp)
add_dependencies(joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy ${catkin_LIBRARIES})
