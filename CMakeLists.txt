cmake_minimum_required(VERSION 2.8.3)
project(mavrosflight)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavrosflight
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# mavrosflight library
add_library(mavrosflight
  src/${PROJECT_NAME}/mavrosflight.cpp
)
add_dependencies(mavrosflight ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavrosflight
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# mavrosflight_node
add_executable(mavrosflight_node
  src/mavrosflight_node.cpp
  src/mavrosflight_ros.cpp
)
add_dependencies(mavrosflight_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavrosflight_node
  mavrosflight
  ${catkin_LIBRARIES}
  ${Boost_LIBRARES}
)

#############
## Install ##
#############

# Mark executables and libraries for installation
install(TARGETS mavrosflight mavrosflight_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

