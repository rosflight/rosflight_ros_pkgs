cmake_minimum_required(VERSION 2.8.3)
project(rosflight_utils)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  rosflight_msgs
  sensor_msgs
  std_srvs
  rosflight
  rosflight_firmware
  rosbag
  async_comm
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
include_directories(${async_comm_INCLUDE_DIRS})

add_library(rosflight_utils
  src/simple_pid.cpp
  src/turbomath.cpp
)
add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosflight_utils ${catkin_LIBRARIES})

add_executable(rosflight_postprocess src/rosflight_postprocess.cpp)
target_link_libraries(rosflight_postprocess ${catkin_LIBRARIES} stdc++fs)

add_executable(viz src/viz.cpp)
add_dependencies(viz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(viz ${catkin_LIBRARIES})

add_library(ublox_lib src/ublox.cpp src/ubx.cpp)

add_executable(ublox_read src/ublox_ros.cpp)
add_dependencies(ublox_read ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ublox_read ${catkin_LIBRARIES} ublox_lib)
