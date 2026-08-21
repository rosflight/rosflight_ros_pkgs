cmake_minimum_required(VERSION 3.20...3.28)
project(rosflight_sim)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(rosflight_compat REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# throw error at cmake time rather than build time if libasio-dev not installed
find_path(asio_INCLUDE_DIR asio.hpp REQUIRED)

##############
## Firmware ##
##############

set(firmware_submodule_dir ${CMAKE_CURRENT_SOURCE_DIR}/../rosflight_firmware)
if(NOT EXISTS ${firmware_submodule_dir}/.git)
  message(FATAL_ERROR
    "Firmware submodule not found at ${firmware_submodule_dir}.\n"
    "You may need to run `git submodule update --init --recursive` "
    "inside of rosflight_ros_pkgs."
  )
endif()

add_subdirectory(
  ${firmware_submodule_dir}
  ${CMAKE_CURRENT_BINARY_DIR}/rosflight_firmware
)

###############
## Libraries ##
###############

# rosflight_sim::interfaces (exported for supported simulators)
add_library(rosflight_sim_interfaces SHARED
  src/time_manager_interface.cpp
  src/forces_and_moments_interface.cpp
  src/sensor_interface.cpp
  src/dynamics_interface.cpp
)
add_library(rosflight_sim::interfaces ALIAS rosflight_sim_interfaces)
set_target_properties(rosflight_sim_interfaces PROPERTIES
  EXPORT_NAME interfaces
  POSITION_INDEPENDENT_CODE ON # needed for gazebo plugin
)
target_compile_features(rosflight_sim_interfaces PUBLIC cxx_std_17)
target_compile_options(rosflight_sim_interfaces PRIVATE -Wall)
target_include_directories(rosflight_sim_interfaces PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(rosflight_sim_interfaces
  PUBLIC
    rclcpp::rclcpp
    std_srvs::std_srvs
    std_msgs::std_msgs
    rosgraph_msgs::rosgraph_msgs
    geometry_msgs::geometry_msgs
    sensor_msgs::sensor_msgs
    rosflight_msgs::rosflight_msgs
    Eigen3::Eigen
  PRIVATE
    rosflight_compat::compat_headers
)

##################
##  Executables ##
##################

# ROSflightSIL
add_executable(rosflight_sil_manager src/rosflight_sil.cpp)
target_compile_features(rosflight_sil_manager PRIVATE cxx_std_17)
target_compile_options(rosflight_sil_manager PRIVATE -Wall)
target_include_directories(rosflight_sil_manager PRIVATE include)
target_link_libraries(rosflight_sil_manager PRIVATE
  rclcpp::rclcpp
  std_srvs::std_srvs
  rosflight_msgs::rosflight_msgs
  rosflight_compat::compat_headers
)

# SILBoard
add_executable(sil_board
  src/sil_board_ros.cpp
  src/sil_board.cpp
  src/udp_board.cpp
)
target_compile_features(sil_board PRIVATE cxx_std_17)
target_compile_options(sil_board PRIVATE -Wall)
target_compile_definitions(sil_board PRIVATE ASIO_STANDALONE)
target_include_directories(sil_board PRIVATE include)
target_link_libraries(sil_board PRIVATE
  rosflight_firmware::rosflight_firmware
  rclcpp::rclcpp
  std_srvs::std_srvs
  sensor_msgs::sensor_msgs
  rosflight_msgs::rosflight_msgs
  Threads::Threads
)

# Standalone simulator
add_subdirectory(simulators/standalone_sim)

# Optional simulator visualizers. These repositories may be cloned into the
# rosflight_sim/simulators directory independently of rosflight_sim.
set(rosfight_simulators_dir ${CMAKE_CURRENT_SOURCE_DIR}/simulators)

set(rosfight_gazebo_classic_dir
  ${rosfight_simulators_dir}/rosflight_viz_gazebo_classic
)
if(EXISTS ${rosfight_gazebo_classic_dir}/CMakeLists.txt)
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    add_subdirectory(simulators/rosflight_viz_gazebo_classic)
  else()
    message(WARNING
      "rosflight_viz_gazebo_classic found, but Gazebo Classic visualizer is "
      "only built on ROS 2 Humble; skipping"
    )
  endif()
endif()

set(rosfight_gazebo_dir ${rosfight_simulators_dir}/rosflight_viz_gazebo)
if(EXISTS ${rosfight_gazebo_dir}/CMakeLists.txt)
  add_subdirectory(simulators/rosflight_viz_gazebo)
else()
  message(WARNING
    "rosflight_viz_gazebo not found; skipping modern Gazebo visualizer"
  )
endif()

set(rosfight_holoocean_dir ${rosfight_simulators_dir}/rosflight_viz_holoocean)
if(EXISTS ${rosfight_holoocean_dir}/CMakeLists.txt)
  add_subdirectory(simulators/rosflight_viz_holoocean)
endif()

#############
## Install ##
#############

# Headers for exported/public libraries
install(
  FILES
    include/rosflight_sim/time_manager_interface.hpp
    include/rosflight_sim/forces_and_moments_interface.hpp
    include/rosflight_sim/sensor_interface.hpp
    include/rosflight_sim/dynamics_interface.hpp
  DESTINATION include/${PROJECT_NAME}/rosflight_sim
)

# Exported/public libraries
install(
  TARGETS
    rosflight_sim_interfaces
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# C++ nodes/executables
install(
  TARGETS
    rosflight_sil_manager
    sil_board
  DESTINATION lib/${PROJECT_NAME}
)

# Python nodes/executables
install(
  PROGRAMS
    src/rc.py
    src/vimfly.py
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(
  DIRECTORY
    launch
    params
    simulators/common_resource
  DESTINATION share/${PROJECT_NAME}
)

########################
### ROS Finalization ###
########################

# Export public libraries
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export public cmake package dependencies (not targets)
ament_export_dependencies(
  rclcpp
  std_srvs
  std_msgs
  rosgraph_msgs
  geometry_msgs
  sensor_msgs
  rosflight_msgs
  Eigen3
)

ament_package()
