cmake_minimum_required(VERSION 3.20...3.28)
project(rosflight_io)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Legacy yaml-cpp support for Ubuntu 22.04
if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
  set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${YAML_CPP_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${YAML_CPP_LIBRARIES}"
  )
endif()

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
  EXCLUDE_FROM_ALL  # prevent compilation of unused targets
)

###############
## Libraries ##
###############

# mavrosflight (private helper for rosflight_io executable - not exported)
add_library(mavrosflight STATIC
  src/mavrosflight/mavrosflight.cpp
  src/mavrosflight/mavlink_comm.cpp
  src/mavrosflight/mavlink_serial.cpp
  src/mavrosflight/mavlink_udp.cpp
  src/mavrosflight/param_manager.cpp
  src/mavrosflight/param.cpp
  src/mavrosflight/time_manager.cpp
)
add_library(rosflight_io::mavrosflight ALIAS mavrosflight)
target_compile_features(mavrosflight PUBLIC cxx_std_17)
target_compile_options(mavrosflight PRIVATE -Wno-address-of-packed-member -Wall)
target_compile_definitions(mavrosflight PUBLIC ASIO_STANDALONE)
target_include_directories(mavrosflight PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:> # intentionally left blank since not exported
)
target_link_libraries(mavrosflight
  PUBLIC
    rosflight_firmware::mavlink_headers
    rclcpp::rclcpp
  PRIVATE
    yaml-cpp::yaml-cpp
    Threads::Threads
)

# realtime_configurator (exported for use in other packages)
add_library(realtime_configurator SHARED src/realtime_configurator.cpp)
add_library(rosflight_io::realtime_configurator ALIAS realtime_configurator)
target_compile_features(realtime_configurator PUBLIC cxx_std_17)
target_compile_options(realtime_configurator PRIVATE -Wall)
target_include_directories(realtime_configurator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(realtime_configurator
  PUBLIC
    rclcpp::rclcpp
    std_msgs::std_msgs
  PRIVATE
    Threads::Threads
)

#################
## Executables ##
#################

# rosflight_io node
add_executable(rosflight_io
  src/rosflight_io_node.cpp
  src/rosflight_io.cpp
  src/magnetometer_calibration.cpp
)
target_compile_features(rosflight_io PRIVATE cxx_std_17)
target_compile_options(rosflight_io PRIVATE -Wno-address-of-packed-member -Wall)
target_include_directories(rosflight_io PRIVATE include)
target_link_libraries(rosflight_io PRIVATE
  rosflight_io::mavrosflight
  rosflight_io::realtime_configurator
  rosflight_firmware::version
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  rosflight_msgs::rosflight_msgs
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  std_srvs::std_srvs
  tf2_geometry_msgs::tf2_geometry_msgs
  Eigen3::Eigen
)

#############
## Install ##
#############

# Headers for exported/public libraries
install(
  FILES include/rosflight_io/realtime_configurator.hpp
  DESTINATION include/${PROJECT_NAME}/rosflight_io
)

# Exported/public libraries
install(
  TARGETS realtime_configurator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# C++ nodes/executables
install(
  TARGETS rosflight_io
  DESTINATION lib/${PROJECT_NAME}
)

##################
# ROS Finalization
##################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export public libraries
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export public cmake package dependencies (not targets)
ament_export_dependencies(
  rclcpp
  std_msgs
)

ament_package()
