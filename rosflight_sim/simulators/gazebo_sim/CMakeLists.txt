cmake_minimum_required(VERSION 3.8)
project(rosflight_sim_gazebo)


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

# Since Gazebo doesn't have an arm64 target, skip this package if gazebo is not found
find_package(gazebo_ros QUIET)
if(NOT gazebo_FOUND)
  install(CODE "message(\"Gazebo not found, skipping ${PROJECT_NAME}\")")
  return()
endif()

# Find packages
find_package(gazebo_dev)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo_ros REQUIRED)

set(INCLUDE_DIRS
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

# TODO: keep going here... Maybe it makes the most sense to have different rosflight_sil (i.e. main) files each with their own cmake?
