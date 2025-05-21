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
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
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

# Time Manager
add_executable(standalone_time_manager
  ../../src/time_manager_interface.cpp
  src/standalone_time_manager.cpp)
target_include_directories(standalone_time_manager
  PRIVATE
    ../../include
    include)
ament_target_dependencies(standalone_time_manager rclcpp std_srvs)
install(
  TARGETS standalone_time_manager
  DESTINATION lib/${PROJECT_NAME})

# Multirotor Forces And Moments
add_executable(multirotor_forces_and_moments
  ../../src/forces_and_moments_interface.cpp
  src/multirotor_forces_and_moments.cpp)
target_include_directories(multirotor_forces_and_moments
  PRIVATE
    ../../include
    include)
ament_target_dependencies(multirotor_forces_and_moments
  rclcpp
  std_srvs
  geometry_msgs
  tf2_eigen
  tf2_geometry_msgs
  rosflight_msgs)
install(
  TARGETS multirotor_forces_and_moments
  DESTINATION lib/${PROJECT_NAME})

# FixedWing Forces And Moments
add_executable(fixedwing_forces_and_moments
  ../../src/forces_and_moments_interface.cpp
  src/fixedwing_forces_and_moments.cpp)
target_include_directories(fixedwing_forces_and_moments
  PRIVATE
    include
    simulators/standalone_sim/include)
ament_target_dependencies(fixedwing_forces_and_moments
  rclcpp
  std_srvs
  geometry_msgs
  tf2_eigen
  tf2_geometry_msgs
  rosflight_msgs)
install(
  TARGETS fixedwing_forces_and_moments
  DESTINATION lib/${PROJECT_NAME})

# Standalone Sensors
add_executable(standalone_sensors
  ../../src/sensor_interface.cpp
  src/standalone_sensors.cpp)
target_include_directories(standalone_sensors
  PRIVATE
    include
    simulators/standalone_sim/include)
target_link_libraries(standalone_sensors Eigen3::Eigen)
ament_target_dependencies(standalone_sensors rclcpp sensor_msgs rosflight_msgs geometry_msgs tf2_eigen)
install(
  TARGETS standalone_sensors
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
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY ../common_resource standalone_resource
  DESTINATION share/${PROJECT_NAME}
)

