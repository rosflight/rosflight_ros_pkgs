cmake_minimum_required(VERSION 2.8.3)
project(rosflight_firmware)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS)

include_directories(firmware/include)
include_directories(firmware/lib/turbotrig)
include_directories(firmware/lib/mavlink)
include_directories(firmware/lib)

catkin_package(
  INCLUDE_DIRS firmware/include
  LIBRARIES rosflight_firmware
)

add_library(rosflight_firmware

 firmware/src/rosflight.cpp
 firmware/src/printf.c
 firmware/src/estimator.cpp
 firmware/src/mixer.cpp
 firmware/src/controller.cpp
 firmware/src/param.cpp
 firmware/src/mode.cpp
 firmware/src/rc.cpp
 firmware/src/mux.cpp
 firmware/src/sensors.cpp

 firmware/lib/turbotrig/turbotrig.c
 firmware/lib/turbotrig/turbovec.c

 firmware/lib/mavlink/mavlink.cpp
 firmware/lib/mavlink/mavlink_param.cpp
 firmware/lib/mavlink/mavlink_stream.cpp
 firmware/lib/mavlink/mavlink_util.cpp
)

install(
  TARGETS rosflight_firmware
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
