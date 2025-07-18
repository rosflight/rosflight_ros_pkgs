cmake_minimum_required(VERSION 3.8)
project(rosflight_io)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

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

# clone firmware submodule if it is missing
set(FIRMWARE_SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../rosflight_firmware")
if(NOT EXISTS "${FIRMWARE_SUBMODULE_DIR}/.git")
  message(STATUS "Firmware submodule not found at ${FIRMWARE_SUBMODULE_DIR}")
  execute_process(
          COMMAND git submodule update --init --recursive
          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

set(FIRMWARE_INCLUDE_DIRS
  ../rosflight_firmware/include/
  ../rosflight_firmware/include/interface/
  ../rosflight_firmware/lib/
  ../rosflight_firmware/comms/
  # ../rosflight_firmware/test/
)

# Determine ROSflight version
execute_process(
  COMMAND git describe --tags --abbrev=8 --always --dirty --long
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_VERSION_STRING RESULT_VARIABLE GIT_RESULT)
if(GIT_RESULT EQUAL 0)
  string(REGEX REPLACE "\n$" "" GIT_VERSION_STRING "${GIT_VERSION_STRING}") # remove trailing newline
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
  ${FIRMWARE_INCLUDE_DIRS}
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
target_compile_options(mavrosflight PRIVATE -Wno-address-of-packed-member)
target_link_libraries(mavrosflight
  ${ament_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  )
ament_target_dependencies(mavrosflight rclcpp)

# rosflight_io_node
add_executable(rosflight_io
  src/rosflight_io_node.cpp
  src/rosflight_io.cpp
  src/magnetometer_calibration.cpp
  )
target_compile_options(rosflight_io PRIVATE -Wno-address-of-packed-member)
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
  Eigen3
  )

#############
## Install ##
#############

# Mark executables and libraries for installation
install(TARGETS mavrosflight rosflight_io
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

# Mark cpp header files for installation
install(DIRECTORY include/rosflight_io/mavrosflight/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )


ament_package()
