cmake_minimum_required(VERSION 3.20...3.28)
project(rosflight_pkgs)

# Acknowledge variable does not need to be used since this file does not define
# an executable (suppresses cmake warning)
if(DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

find_package(ament_cmake REQUIRED)

ament_package()
