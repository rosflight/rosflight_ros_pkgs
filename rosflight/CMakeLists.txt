cmake_minimum_required(VERSION 3.5)
project(rosflight)

# TODO: Cleanup/rewrite file

set(CMAKE_BUILD_TYPE Release)

message("CMAKE_C_FLAGS_RELEASE is ${CMAKE_C_FLAGS_RELEASE}")

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

## Look for and clone MAVLINK if it is missing
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/rosflight/mavlink/v1.0/.git")
  message(STATUS "MAVLink submodule not found at ${CMAKE_CURRENT_SOURCE_DIR}/include/rosflight/mavlink/v1.0")
  execute_process(
    COMMAND git submodule update --init --recursive
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

execute_process(COMMAND git describe --tags --abbrev=8 --always --dirty --long WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} OUTPUT_VARIABLE GIT_VERSION_STRING RESULT_VARIABLE GIT_RESULT)
if(GIT_RESULT EQUAL 0)
  string(REGEX REPLACE "\n$" "" GIT_VERSION_STRING "${GIT_VERSION_STRING}") # remove trailing newline
  message("GIT_VERSION_STRING: ${GIT_VERSION_STRING}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROSFLIGHT_VERSION=${GIT_VERSION_STRING}")
else()
  message("Could not determine rosflight version through git")
endif()


###########
## Build ##
###########

include_directories(include
  ${ament_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)
install(
  DIRECTORY include
  DESTINATION include
)

# mavrosflight library
add_library(mavrosflight
  src/mavrosflight/mavrosflight.cpp
  src/mavrosflight/mavlink_comm.cpp
  src/mavrosflight/mavlink_serial.cpp
  src/mavrosflight/mavlink_udp.cpp
  src/mavrosflight/param_manager.cpp
  src/mavrosflight/param.cpp
  src/mavrosflight/time_manager.cpp
)
ament_target_dependencies(mavrosflight rclcpp)
ament_export_dependencies(rclcpp
  eigen_stl_containers
  geometry_msgs
  rosflight_msgs
  sensor_msgs
  std_msgs
  Boost
  EIGEN3
  YAML_CPP
)
ament_export_include_directories(include)
ament_export_libraries(mavrosflight)
target_compile_definitions(mavrosflight PRIVATE USE_ROS)
target_link_libraries(mavrosflight
  ${ament_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# rosflight_io_node
add_executable(rosflight_io
  src/rosflight_io_node.cpp
  src/rosflight_io.cpp
)
target_link_libraries(rosflight_io
  mavrosflight
  ${rclcpp_LIBRARIES}
  ${ament_LIBRARIES}
  ${Boost_LIBRARES}
)
ament_target_dependencies(rosflight_io
  geometry_msgs
  rosflight_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)

# calibrate mag node
add_executable(calibrate_mag
  src/mag_cal_node.cpp
  src/mag_cal.cpp
)
target_link_libraries(calibrate_mag
  ${rclcpp_LIBRARIES}
  ${ament_LIBRARIES}
  ${Boost_LIBRARIES}
)
ament_target_dependencies(calibrate_mag
  rosflight_msgs
  sensor_msgs
  eigen_stl_containers
  message_filters
)


#############
## Install ##
#############

# Mark executables and libraries for installation
install(TARGETS mavrosflight rosflight_io calibrate_mag
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/rosflight/mavrosflight/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


ament_package(
  # CONFIG_EXTRAS
)
