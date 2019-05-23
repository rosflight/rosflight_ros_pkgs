cmake_minimum_required(VERSION 2.8.3)
project(rosflight)

set(CMAKE_BUILD_TYPE Release)

message("CMAKE_C_FLAGS_RELEASE is ${CMAKE_C_FLAGS_RELEASE}")

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_stl_containers
  geometry_msgs
  rosflight_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
)
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

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavrosflight
  CATKIN_DEPENDS roscpp eigen_stl_containers geometry_msgs rosflight_msgs sensor_msgs std_msgs tf
  DEPENDS Boost EIGEN3 YAML_CPP tf
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
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
add_dependencies(mavrosflight ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavrosflight
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# rosflight_io_node
add_executable(rosflight_io
  src/rosflight_io_node.cpp
  src/rosflight_io.cpp
)
add_dependencies(rosflight_io ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosflight_io
  mavrosflight
  ${catkin_LIBRARIES}
  ${Boost_LIBRARES}
)

add_executable(calibrate_mag
    src/mag_cal_node.cpp
    src/mag_cal.cpp
)
add_dependencies(calibrate_mag ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(calibrate_mag
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and libraries for installation
install(TARGETS mavrosflight rosflight_io
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/rosflight/mavrosflight/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
