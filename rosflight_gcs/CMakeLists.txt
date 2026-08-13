cmake_minimum_required(VERSION 3.20...3.28)
project(rosflight_gcs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#################
## Executables ##
#################

# viz executable
add_executable(viz src/viz.cpp)
target_compile_features(viz PRIVATE cxx_std_17)
target_compile_options(viz PRIVATE -Wall)
target_include_directories(viz PRIVATE include)
target_link_libraries(viz PRIVATE
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  rosflight_msgs::rosflight_msgs
  sensor_msgs::sensor_msgs
  tf2_geometry_msgs::tf2_geometry_msgs
  visualization_msgs::visualization_msgs
)

#############
## Install ##
#############

# C++ nodes/executables
install(
  TARGETS viz
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(
  DIRECTORY
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

######################
## ROS Finalization ##
######################

ament_package()
