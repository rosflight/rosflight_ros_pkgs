cmake_minimum_required(VERSION 3.8)
project(rosflight_sim)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)


##############
## Firmware ##
##############

# clone firmware submodule if it is missing
set(FIRMWARE_SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../rosflight_firmware")
if(NOT EXISTS "${FIRMWARE_SUBMODULE_DIR}/.git")
  message(STATUS "Firmware submodule not found at ${FIRMWARE_SUBMODULE_DIR}")
  execute_process(
          COMMAND git submodule update --init --recursive
          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

set(FIRMWARE_INCLUDE_DIRS
  ../rosflight_firmware/include/
  ../rosflight_firmware/include/interface/
  ../rosflight_firmware/lib/
  ../rosflight_firmware/comms/
  ../rosflight_firmware/test/
)

include_directories(
  ${FIRMWARE_INCLUDE_DIRS}
)

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

# rosflight_firmware
add_library(rosflight_firmware
  ${FIRMWARE_SUBMODULE_DIR}/src/rosflight.cpp
  ${FIRMWARE_SUBMODULE_DIR}/src/estimator.cpp
  ${FIRMWARE_SUBMODULE_DIR}/src/mixer.cpp
  ${FIRMWARE_SUBMODULE_DIR}/src/controller.cpp
  ${FIRMWARE_SUBMODULE_DIR}/src/param.cpp
  ${FIRMWARE_SUBMODULE_DIR}/src/state_manager.cpp
  ${FIRMWARE_SUBMODULE_DIR}/src/rc.cpp
  ${FIRMWARE_SUBMODULE_DIR}/src/command_manager.cpp
  ${FIRMWARE_SUBMODULE_DIR}/src/sensors.cpp
  ${FIRMWARE_SUBMODULE_DIR}/src/comm_manager.cpp
  ${FIRMWARE_SUBMODULE_DIR}/comms/mavlink/mavlink.cpp
  ${FIRMWARE_SUBMODULE_DIR}/lib/turbomath/turbomath.cpp
)
target_include_directories(rosflight_firmware PRIVATE ${FIRMWARE_INCLUDE_DIRS})
target_link_libraries(rosflight_firmware Eigen3::Eigen)
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


###################
## ROSflight SIL ##
###################

include_directories(include
  ${ament_INCLUDE_DIRS}
  ${rclcpp_INLCUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

##################
##  Executables ##
##################

# ROSflightSIL
add_executable(rosflight_sil_manager
  src/rosflight_sil.cpp)
target_include_directories(rosflight_sil_manager
  PRIVATE include)
ament_target_dependencies(rosflight_sil_manager rclcpp std_srvs rosflight_msgs)
install(
  TARGETS rosflight_sil_manager
  DESTINATION lib/${PROJECT_NAME})

# SILBoard
add_executable(sil_board
  src/sil_board_ros.cpp
  src/sil_board.cpp
  src/udp_board.cpp)
target_link_libraries(sil_board
  rosflight_firmware
  ${Boost_LIBRARIES})
target_include_directories(sil_board
  PRIVATE
    include)
ament_target_dependencies(sil_board rclcpp std_srvs sensor_msgs rosflight_msgs)
install(
  TARGETS sil_board
  DESTINATION lib/${PROJECT_NAME})

# Standalone simulator
add_subdirectory(simulators/standalone_sim)

# Optional simulator visualizers. These repositories may be cloned into the
# rosflight_sim/simulators directory independently of rosflight_sim.
set(ROSFIGHT_SIMULATORS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/simulators")

set(ROSFIGHT_GAZEBO_CLASSIC_DIR "${ROSFIGHT_SIMULATORS_DIR}/rosflight_viz_gazebo_classic")
if(EXISTS "${ROSFIGHT_GAZEBO_CLASSIC_DIR}/CMakeLists.txt")
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    add_subdirectory(simulators/rosflight_viz_gazebo_classic)
  else()
    message(WARNING
      "rosflight_viz_gazebo_classic found, but Gazebo Classic visualizer is only built on ROS 2 Humble; skipping")
  endif()
endif()

set(ROSFIGHT_GAZEBO_DIR "${ROSFIGHT_SIMULATORS_DIR}/rosflight_viz_gazebo")
if(EXISTS "${ROSFIGHT_GAZEBO_DIR}/CMakeLists.txt")
  add_subdirectory(simulators/rosflight_viz_gazebo)
else()
  message(WARNING "rosflight_viz_gazebo not found; skipping modern Gazebo visualizer")
endif()

set(ROSFIGHT_HOLOOCEAN_DIR "${ROSFIGHT_SIMULATORS_DIR}/rosflight_viz_holoocean")
if(EXISTS "${ROSFIGHT_HOLOOCEAN_DIR}/CMakeLists.txt")
  add_subdirectory(simulators/rosflight_viz_holoocean)
endif()

#############
## Install ##
#############

install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/rosflight_sim
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/rc.py
  src/vimfly.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
