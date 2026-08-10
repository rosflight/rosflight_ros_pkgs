cmake_minimum_required(VERSION 3.20...3.28)
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
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(rosflight_compat REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS thread)


##############
## Firmware ##
##############

set(firmware_submodule_dir "${CMAKE_CURRENT_SOURCE_DIR}/../rosflight_firmware")

# clone firmware submodule if it is missing
if(NOT EXISTS "${firmware_submodule_dir}/.git")
  message(STATUS "Firmware submodule not found at ${firmware_submodule_dir}")
  execute_process(
    COMMAND git submodule update --init --recursive
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

add_subdirectory(
  "${firmware_submodule_dir}"
  "${CMAKE_CURRENT_BINARY_DIR}/rosflight_firmware"
)

###############
## Libraries ##
###############

)
)
)

###################
## ROSflight SIL ##
###################

include_directories(include
  ${SDFormat_INCLUDE_DIRS}
)

##################
##  Executables ##
##################

# ROSflightSIL
add_executable(rosflight_sil_manager src/rosflight_sil.cpp)
target_include_directories(rosflight_sil_manager PRIVATE include)
target_link_libraries(rosflight_sil_manager PRIVATE
  rclcpp::rclcpp
  std_srvs::std_srvs
  rosflight_msgs::rosflight_msgs
  rosflight_compat::compat_headers
)
install(
  TARGETS rosflight_sil_manager
  DESTINATION lib/${PROJECT_NAME})

# SILBoard
add_executable(sil_board
  src/sil_board_ros.cpp
  src/sil_board.cpp
  src/udp_board.cpp
)
target_include_directories(sil_board PRIVATE include)
target_link_libraries(sil_board PRIVATE
  rosflight_firmware::rosflight_firmware
  rclcpp::rclcpp
  std_srvs::std_srvs
  sensor_msgs::sensor_msgs
  rosflight_msgs::rosflight_msgs
  Boost::thread
)
install(
  TARGETS sil_board
  DESTINATION lib/${PROJECT_NAME}
)

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
