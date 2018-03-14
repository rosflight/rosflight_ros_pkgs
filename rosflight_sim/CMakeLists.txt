cmake_minimum_required(VERSION 2.8.3)
project(rosflight_sim)

find_package(gazebo)
IF(gazebo_FOUND)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Default to release build for speed
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  rosflight_firmware
  rosflight_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp gazebo_plugins gazebo_ros geometry_msgs rosflight_firmware rosflight_msgs
  DEPENDS EIGEN3 GAZEBO
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(rosflight_sil_plugin SHARED
  src/rosflight_sil.cpp
  src/sil_board.cpp
  src/multirotor_forces_and_moments.cpp
  src/fixedwing_forces_and_moments.cpp
)
target_link_libraries(rosflight_sil_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(rosflight_sil_plugin ${catkin_EXPORTED_TARGETS})

install(
  TARGETS rosflight_sil_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY xacro/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/xacro
)

install(
  DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
)

ENDIF()
