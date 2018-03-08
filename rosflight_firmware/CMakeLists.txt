cmake_minimum_required(VERSION 2.8.3)
project(rosflight_firmware)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Boost REQUIRED COMPONENTS system thread)

# clone firmware submodule if it is missing
set(FIRMWARE_SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/firmware")
if(NOT EXISTS "${FIRMWARE_SUBMODULE_DIR}/.git")
  message(STATUS "Firmware submodule not found at ${FIRMWARE_SUBMODULE_DIR}")
  execute_process(
    COMMAND git submodule update --init --recursive
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

set(FIRMWARE_INCLUDE_DIRS
  firmware/include/
  firmware/lib/
  firmware/comms/
)

catkin_package(
  INCLUDE_DIRS include ${FIRMWARE_INCLUDE_DIRS}
  LIBRARIES rosflight_firmware rosflight_udp_board
  CATKIN_DEPENDS roscpp
  CFG_EXTRAS rosflight_firmware-extras.cmake
)

include_directories(include)
include_directories(${FIRMWARE_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

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
  firmware/src/comm_manager.cpp

  firmware/comms/mavlink/mavlink.cpp

  firmware/lib/turbomath/turbomath.cpp
)

add_library(rosflight_udp_board
  src/udp_board.cpp
)
target_link_libraries(rosflight_udp_board
  rosflight_firmware
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS rosflight_firmware rosflight_udp_board
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/ ${FIRMWARE_INCLUDE_DIRS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
