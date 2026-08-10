cmake_minimum_required(VERSION 3.20...3.28)
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
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(Boost CONFIG REQUIRED COMPONENTS thread)
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

# clone firmware submodule if it is missing
set(firmware_submodule_dir "${CMAKE_CURRENT_SOURCE_DIR}/../rosflight_firmware")
if(NOT EXISTS "${firmware_submodule_dir}/.git")
  message(STATUS "Firmware submodule not found at ${firmware_submodule_dir}")
  execute_process(
    COMMAND git submodule update --init --recursive
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

add_subdirectory(
  "${firmware_submodule_dir}"
  "${CMAKE_CURRENT_BINARY_DIR}/rosflight_firmware"
  EXCLUDE_FROM_ALL  # prevent compilation of unused targets
)

###############
## Libraries ##
###############

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
target_link_libraries(mavrosflight PRIVATE
  rclcpp::rclcpp
  yaml-cpp::yaml-cpp
  Boost::headers
  Boost::thread
)

# realtime_configurator library
add_library(realtime_configurator SHARED
  src/realtime_configurator.cpp
)
target_include_directories(realtime_configurator PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(realtime_configurator PRIVATE
  rclcpp::rclcpp
  std_msgs::std_msgs
)
ament_export_targets(export_realtime_configurator HAS_LIBRARY_TARGET)

# rosflight_io_node
add_executable(rosflight_io
  src/rosflight_io_node.cpp
  src/rosflight_io.cpp
  src/magnetometer_calibration.cpp
)
target_compile_options(rosflight_io PRIVATE -Wno-address-of-packed-member)
target_link_libraries(rosflight_io PRIVATE
  mavrosflight
  realtime_configurator
  rosflight_firmware::version
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  rosflight_msgs::rosflight_msgs
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  std_srvs::std_srvs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  Boost::headers
  Eigen3::Eigen
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

install(TARGETS realtime_configurator
  EXPORT export_realtime_configurator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(FILES include/rosflight_io/realtime_configurator.hpp
  DESTINATION include
)

ament_package()
