cmake_minimum_required(VERSION 2.8.3)
project(fcu_io)

find_package(catkin REQUIRED COMPONENTS
  fcu_common
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  cmake_modules
)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

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
  src/mavrosflight/mavrosflight.cpp
  src/mavrosflight/mavlink_serial.cpp
  src/mavrosflight/param_manager.cpp
  src/mavrosflight/param.cpp
  src/mavrosflight/time_manager.cpp
  src/mavrosflight/sensors/differential_pressure.cpp
  src/mavrosflight/sensors/imu.cpp
  src/mavrosflight/sensors/baro.cpp
)
add_dependencies(mavrosflight ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavrosflight
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# fcu_io_node
add_executable(fcu_io_node
  src/fcu_io_node.cpp
  src/fcu_io.cpp
)
add_dependencies(fcu_io_node fcu_common_generate_messages_cpp)
add_dependencies(fcu_io_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fcu_io_node
  mavrosflight
  ${catkin_LIBRARIES}
  ${Boost_LIBRARES}
)

#############
## Install ##
#############

# Mark executables and libraries for installation
install(TARGETS mavrosflight fcu_io_node
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
