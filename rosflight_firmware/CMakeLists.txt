cmake_minimum_required(VERSION 3.5)
project(rosflight_firmware)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -fPIC)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

# Git information
execute_process(COMMAND git rev-parse --short=8 HEAD
    OUTPUT_VARIABLE GIT_VERSION_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/firmware)

execute_process(COMMAND git describe --tags --abbrev=8 --always --dirty --long
    OUTPUT_VARIABLE GIT_VERSION_STRING
    OUTPUT_STRIP_TRAILING_WHITESPACE
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/firmware)

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
  firmware/include/
  firmware/lib/
  firmware/comms/
  firmware/test/
)

include_directories(include
  ${FIRMWARE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# rosflight_firmware
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
  src/udp_board.cpp
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
    firmware/test/test_board.cpp
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
  DIRECTORY include/${PROJECT_NAME}/ ${FIRMWARE_INCLUDE_DIRS}
  DESTINATION include
)


ament_package()
