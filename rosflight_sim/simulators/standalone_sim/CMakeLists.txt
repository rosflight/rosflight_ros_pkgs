cmake_minimum_required(VERSION 3.8)
# Don't define own project -- use parent instead

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -fPIC)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Rosflight Standalone sim
add_executable(rviz_sim_publisher
  src/standalone_sim.cpp)
target_include_directories(rviz_sim_publisher
  PRIVATE
    include)
ament_target_dependencies(rviz_sim_publisher
  geometry_msgs
  rclcpp
  rosflight_msgs
  tf2
  tf2_ros
  visualization_msgs)
ament_export_dependencies(rclcpp
  tf2
  tf2_ros
  geometry_msgs
  rosflight_msgs
  visualization_msgs)
install(
  TARGETS rviz_sim_publisher
  DESTINATION lib/${PROJECT_NAME})

# Dynamics
add_executable(standalone_dynamics
  ../../src/dynamics_interface.cpp
  src/standalone_dynamics.cpp)
target_include_directories(standalone_dynamics
  PRIVATE
    ../../include
    include)
target_link_libraries(standalone_dynamics
  Eigen3::Eigen)
ament_target_dependencies(standalone_dynamics
  geometry_msgs
  tf2_eigen
  tf2_geometry_msgs
  std_srvs
  rclcpp
  rosflight_msgs)
ament_export_dependencies(standalone_dynamics
  geometry_msgs
  tf2_eigen
  tf2_geometry_msgs
  std_srvs
  rclcpp
  rosflight_msgs)
install(
  TARGETS standalone_dynamics
  DESTINATION lib/${PROJECT_NAME})

# Other install
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY ../common_resource
  DESTINATION share/${PROJECT_NAME}
)

