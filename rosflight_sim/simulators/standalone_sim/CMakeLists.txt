cmake_minimum_required(VERSION 3.20...3.28)
# Don't define own project -- use parent instead

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

#################
## EXECUTABLES ##
#################

# Viz Transcriber
add_executable(standalone_viz_transcriber src/standalone_viz_transcriber.cpp)
target_compile_features(standalone_viz_transcriber PRIVATE cxx_std_17)
target_compile_options(standalone_viz_transcriber PRIVATE -Wall)
target_include_directories(standalone_viz_transcriber PRIVATE include)
target_link_libraries(standalone_viz_transcriber PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  rosflight_msgs::rosflight_msgs
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  visualization_msgs::visualization_msgs
)

# Time Manager
add_executable(standalone_time_manager src/standalone_time_manager.cpp)
target_compile_features(standalone_time_manager PRIVATE cxx_std_17)
target_compile_options(standalone_time_manager PRIVATE -Wall)
target_include_directories(standalone_time_manager PRIVATE include)
target_link_libraries(standalone_time_manager PRIVATE rosflight_sim::interfaces)

# Multirotor Forces And Moments
add_executable(multirotor_forces_and_moments
  src/multirotor_forces_and_moments.cpp
)
target_compile_features(multirotor_forces_and_moments PRIVATE cxx_std_17)
target_compile_options(multirotor_forces_and_moments PRIVATE -Wall)
target_include_directories(multirotor_forces_and_moments PRIVATE include)
target_link_libraries(multirotor_forces_and_moments PRIVATE
  rosflight_sim::interfaces
  geometry_msgs::geometry_msgs
  rosflight_msgs::rosflight_msgs
  Eigen3::Eigen
)

# FixedWing Forces And Moments
add_executable(fixedwing_forces_and_moments
  src/fixedwing_forces_and_moments.cpp
)
target_compile_features(fixedwing_forces_and_moments PRIVATE cxx_std_17)
target_compile_options(fixedwing_forces_and_moments PRIVATE -Wall)
target_include_directories(fixedwing_forces_and_moments PRIVATE include)
target_link_libraries(fixedwing_forces_and_moments PRIVATE
  rosflight_sim::interfaces
  geometry_msgs::geometry_msgs
  rosflight_msgs::rosflight_msgs
  Eigen3::Eigen
)

# Standalone Sensors
add_executable(standalone_sensors src/standalone_sensors.cpp)
target_compile_features(standalone_sensors PRIVATE cxx_std_17)
target_compile_options(standalone_sensors PRIVATE -Wall)
target_include_directories(standalone_sensors PRIVATE include)
target_link_libraries(standalone_sensors PRIVATE
  rosflight_sim::interfaces
  rclcpp::rclcpp
  rosflight_msgs::rosflight_msgs
  geometry_msgs::geometry_msgs
  Eigen3::Eigen
)

# Dynamics
add_executable(standalone_dynamics src/standalone_dynamics.cpp)
target_compile_features(standalone_dynamics PRIVATE cxx_std_17)
target_compile_options(standalone_dynamics PRIVATE -Wall)
target_include_directories(standalone_dynamics PRIVATE include)
target_link_libraries(standalone_dynamics PRIVATE
  rosflight_sim::interfaces
  rclcpp::rclcpp
  rosflight_msgs::rosflight_msgs
  geometry_msgs::geometry_msgs
  tf2_geometry_msgs::tf2_geometry_msgs
  Eigen3::Eigen
)

#############
## Install ##
#############

# C++ node executables
install(
  TARGETS
    standalone_viz_transcriber
    standalone_time_manager
    multirotor_forces_and_moments
    fixedwing_forces_and_moments
    standalone_sensors
    standalone_dynamics
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(
  DIRECTORY
    launch
    params
    config
    standalone_resource
  DESTINATION share/${PROJECT_NAME}
)

