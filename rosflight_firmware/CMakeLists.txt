cmake_minimum_required(VERSION 2.8.3)
project(rosflight_firmware)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS)

include_directories(firmware/include)
include_directories(firmware/lib/turbotrig)
include_directories(firmware/lib/mavlink)
include_directories(firmware/lib)

catkin_package(
  INCLUDE_DIRS firmware/include firmware/lib/turbotrig firmware/lib/mavlink firmware/lib
  LIBRARIES rosflight_firmware
)

add_library(rosflight_firmware

 firmware/src/rosflight.cpp
 firmware/src/nanoprintf.cpp
 firmware/src/estimator.cpp
 firmware/src/mixer.cpp
 firmware/src/controller.cpp
 firmware/src/param.cpp
 firmware/src/state_manager.cpp
 firmware/src/rc.cpp
 firmware/src/command_manager.cpp
 firmware/src/sensors.cpp

 firmware/src/mavlink.cpp

 firmware/lib/turbotrig/turbotrig.cpp
 firmware/lib/turbotrig/turbovec.cpp
)

install(
  TARGETS rosflight_firmware
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
