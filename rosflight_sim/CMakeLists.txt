cmake_minimum_required(VERSION 2.8.3)
project(rosflight_sim)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  rosbag
  roscpp
  rosflight_msgs
  rosflight_utils
  rosflight_firmware
  gazebo_ros
  gazebo_plugins
  geometry_msgs
)

find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  CATKIN_DEPENDS rosbag roscpp roscpp gazebo_plugins gazebo_ros
  DEPENDS Eigen gazebo opencv rosflight_msgs SDF roscpp gazebo_ros 
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(lib/SIL_board)

add_library(ROSflight_sil_plugin
 src/ROSflight_sil.cpp
 src/SIL_board.cpp
 src/multirotor_forces_and_moments.cpp
 src/fixedwing_forces_and_moments.cpp
 )
target_link_libraries(ROSflight_sil_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} yaml-cpp)
add_dependencies(ROSflight_sil_plugin ${catkin_EXPORTED_TARGETS} rosflight_msgs_generate_messages_cpp)
set_property( TARGET ROSflight_sil_plugin APPEND_STRING PROPERTY COMPILE_FLAGS -Wno-format-extra-args )

#install(
#  TARGETS
#    ROSflight_sil_plugin
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)
