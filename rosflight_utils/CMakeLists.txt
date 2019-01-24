cmake_minimum_required(VERSION 2.8.3)
project(rosflight_utils)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  rosflight_msgs
  sensor_msgs
  std_srvs
  visualization_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosflight_utils
  CATKIN_DEPENDS roscpp rospy rosflight_msgs sensor_msgs std_srvs visualization_msgs
)

add_definitions(-std=c++11)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(rosflight_utils
  src/simple_pid.cpp
  src/turbomath.cpp
)
add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosflight_utils ${catkin_LIBRARIES})

#add_executable(joy src/joy.cpp)
#add_dependencies(joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(joy ${catkin_LIBRARIES})

add_executable(viz src/viz.cpp)
add_dependencies(viz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(viz ${catkin_LIBRARIES})
