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
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
# find_package(nav_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)


##############
## Firmware ##
##############

set(FIRMWARE_INCLUDE_DIRS
  include/rosflight_sim/rosflight_firmware/include/
  include/rosflight_sim/rosflight_firmware/lib/
  include/rosflight_sim/rosflight_firmware/comms/
  include/rosflight_sim/rosflight_firmware/test/
)

include_directories(
  ${FIRMWARE_INCLUDE_DIRS}
)

# clone firmware submodule if it is missing
set(FIRMWARE_SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/rosflight_sim/rosflight_firmware")
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
  include/rosflight_sim/rosflight_firmware/src/rosflight.cpp
  include/rosflight_sim/rosflight_firmware/src/estimator.cpp
  include/rosflight_sim/rosflight_firmware/src/mixer.cpp
  include/rosflight_sim/rosflight_firmware/src/controller.cpp
  include/rosflight_sim/rosflight_firmware/src/param.cpp
  include/rosflight_sim/rosflight_firmware/src/state_manager.cpp
  include/rosflight_sim/rosflight_firmware/src/rc.cpp
  include/rosflight_sim/rosflight_firmware/src/command_manager.cpp
  include/rosflight_sim/rosflight_firmware/src/sensors.cpp
  include/rosflight_sim/rosflight_firmware/src/comm_manager.cpp
  include/rosflight_sim/rosflight_firmware/comms/mavlink/mavlink.cpp
  include/rosflight_sim/rosflight_firmware/lib/turbomath/turbomath.cpp
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
  #${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
#link_directories(${GAZEBO_LIBRARY_DIRS})

##################
##  Executables ##
##################

# Time Manager
add_executable(time_manager_standalone
  src/time_manager_interface.cpp
  simulators/standalone_sim/src/standalone_time_manager.cpp)
target_include_directories(time_manager_standalone
  PRIVATE 
    include
    simulators/standalone_sim/include)
ament_target_dependencies(time_manager_standalone rclcpp)
install(
  TARGETS time_manager_standalone
  DESTINATION lib/${PROJECT_NAME})

# ROSflightSIL
add_executable(rosflight_sil
  src/rosflight_sil.cpp)
target_include_directories(rosflight_sil
  PRIVATE include)
ament_target_dependencies(rosflight_sil rclcpp std_srvs rosflight_msgs)
install(
  TARGETS rosflight_sil
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
ament_target_dependencies(sil_board rclcpp sensor_msgs rosflight_msgs)
install(
  TARGETS sil_board
  DESTINATION lib/${PROJECT_NAME})

# Standalone Sensors
add_executable(standalone_sensors
  src/sensor_interface.cpp
  simulators/standalone_sim/src/standalone_sensors.cpp)
target_include_directories(standalone_sensors
  PRIVATE
    include
    simulators/standalone_sim/include)
target_link_libraries(standalone_sensors Eigen3::Eigen)
ament_target_dependencies(standalone_sensors rclcpp sensor_msgs rosflight_msgs geometry_msgs tf2_eigen) 
install(
  TARGETS standalone_sensors
  DESTINATION lib/${PROJECT_NAME})


# add_library(rosflight_sil_plugin SHARED
#   src/rosflight_sil.cpp
#   src/sil_board.cpp
#   src/udp_board.cpp
#   src/multirotor_forces_and_moments.cpp
#   src/fixedwing_forces_and_moments.cpp
# )
# target_link_libraries(rosflight_sil_plugin
#   rosflight_firmware
#   ${rclcpp_INLCUDE_DIRS}
#   ${ament_INCLUDE_DIRS}
#   ${GAZEBO_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# ament_target_dependencies(rosflight_sil_plugin
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   rosflight_msgs
#   gazebo_ros
# )
# ament_export_targets(rosflight_sil_pluginTargets HAS_LIBRARY_TARGET)
# ament_export_dependencies(
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   rosflight_msgs
#   gazebo_ros
# )
# 
# install(
#   TARGETS rosflight_sil_plugin
#   EXPORT rosflight_sil_pluginTargets
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )


#############
## Install ##
#############

# install(
#   DIRECTORY include/${PROJECT_NAME}
#   DESTINATION include/${PROJECT_NAME}
# )
# 
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )
# 
# install(
#   DIRECTORY simulators/gazebo/xacro
#   DESTINATION share/${PROJECT_NAME}
# )
# 
# install(
#   DIRECTORY simulators/gazebo/resources
#   DESTINATION share/${PROJECT_NAME}
# )
# 
# install(
#   DIRECTORY params
#   DESTINATION share/${PROJECT_NAME}
# )
# 
# install(PROGRAMS
#   src/rc.py
#   src/vimfly.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
