cmake_minimum_required(VERSION 3.8)
project(rosflight_firmware)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -fPIC)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# clone rosflight_firmware submodule if it is missing
set(FIRMWARE_SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rosflight_firmware")
if(NOT EXISTS "${FIRMWARE_SUBMODULE_DIR}/.git")
  message(STATUS "rosflight_firmware submodule not found at ${FIRMWARE_SUBMODULE_DIR}")
  execute_process(
    COMMAND git submodule update --init --recursive
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

# Git information
execute_process(COMMAND git rev-parse --short=8 HEAD
  OUTPUT_VARIABLE GIT_VERSION_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rosflight_firmware)

execute_process(COMMAND git describe --tags --abbrev=8 --always --dirty --long
  OUTPUT_VARIABLE GIT_VERSION_STRING
  OUTPUT_STRIP_TRAILING_WHITESPACE
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rosflight_firmware)

if("${GIT_VERSION_STRING}" STREQUAL "")
  set(GIT_VERSION_STRING "undefined")
endif()

if("${GIT_VERSION_HASH}" STREQUAL "")
  set(GIT_VERSION_HASH "0")
endif()


###########
## Build ##
###########

set(FIRMWARE_INCLUDE_DIRS
  rosflight_firmware/include/
  rosflight_firmware/lib/
  rosflight_firmware/comms/
  rosflight_firmware/test/
  rosflight_firmware/boards/udp/
  )

include_directories(include
  ${FIRMWARE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

# rosflight_firmware
add_library(rosflight_firmware
  rosflight_firmware/src/rosflight.cpp
  rosflight_firmware/src/nanoprintf.cpp
  rosflight_firmware/src/estimator.cpp
  rosflight_firmware/src/mixer.cpp
  rosflight_firmware/src/controller.cpp
  rosflight_firmware/src/param.cpp
  rosflight_firmware/src/state_manager.cpp
  rosflight_firmware/src/rc.cpp
  rosflight_firmware/src/command_manager.cpp
  rosflight_firmware/src/sensors.cpp
  rosflight_firmware/src/comm_manager.cpp

  rosflight_firmware/comms/mavlink/mavlink.cpp

  rosflight_firmware/lib/turbomath/turbomath.cpp
  )
target_compile_definitions(rosflight_firmware PUBLIC
  GIT_VERSION_HASH=0x${GIT_VERSION_HASH}
  GIT_VERSION_STRING=\"${GIT_VERSION_STRING}\"
  )
ament_export_targets(rosflight_firmwareTargets HAS_LIBRARY_TARGET)
install(
  TARGETS rosflight_firmware
  EXPORT rosflight_firmwareTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# rosflight_udp_board
add_library(rosflight_udp_board
  rosflight_firmware/boards/udp/udp_board.cpp
  )
target_link_libraries(rosflight_udp_board
  rosflight_firmware
  ${rclcpp_LIBRARIES}
  ${ament_LIBRARIES}
  ${Boost_LIBRARIES}
  )
ament_export_targets(rosflight_udp_boardTargets HAS_LIBRARY_TARGET)
install(
  TARGETS rosflight_udp_board
  EXPORT rosflight_udp_boardTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# rosflight_test
add_library(rosflight_test
  rosflight_firmware/test/test_board.cpp
  )
target_link_libraries(rosflight_test
  rosflight_firmware
  )
ament_export_targets(rosflight_testTargets HAS_LIBRARY_TARGET)
install(
  TARGETS rosflight_test
  EXPORT rosflight_testTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


install(
  DIRECTORY ${PROJECT_NAME}/ ${FIRMWARE_INCLUDE_DIRS}
  DESTINATION include
)


ament_package()
