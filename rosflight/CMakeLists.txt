cmake_minimum_required(VERSION 2.8.3)
project(rosflight)

# Use this to make Eigen run REALLY fast
set(CMAKE_CXX_FLAGS "-std=c++0x -fopenmp -O3")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  rosflight_common
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen REQUIRED)
pkg_check_modules(YAML yaml-cpp)

###################################
## catkin specific configuration ##
###################################
#
# add_message_files(
#   FILES
#   Command.msg
#   ServoOutputRaw.msg
# )

add_service_files(
  FILES
  ParamFile.srv
  ParamGet.srv
  ParamSet.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavrosflight
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  DEPENDS Boost Eigen yaml-cpp
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${YAML_INCLUDEDIR}
)

# mavrosflight library
add_library(mavrosflight
  src/mavrosflight/mavrosflight.cpp
  src/mavrosflight/mavlink_serial.cpp
  src/mavrosflight/param_manager.cpp
  src/mavrosflight/param.cpp
  src/mavrosflight/time_manager.cpp
  src/mavrosflight/sensors/imu.cpp
  src/mavrosflight/sensors/mag.cpp
)
add_dependencies(mavrosflight ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavrosflight
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
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
install(DIRECTORY include/mavrosflight/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
