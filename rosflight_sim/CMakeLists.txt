cmake_minimum_required(VERSION 3.8)
project(rosflight_sim)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosflight_firmware REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include
  ${ament_INCLUDE_DIRS}
  ${rclcpp_INLCUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  )
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(rosflight_sil_plugin SHARED
  src/rosflight_sil.cpp
  src/sil_board.cpp
  src/multirotor_forces_and_moments.cpp
  src/fixedwing_forces_and_moments.cpp
  )
target_link_libraries(rosflight_sil_plugin
  ${rclcpp_INLCUDE_DIRS}
  ${ament_INCLUDE_DIRS}
  ${GAZEBO_LIBRARIES}
  )
ament_target_dependencies(rosflight_sil_plugin
  rclcpp
  geometry_msgs
  nav_msgs
  rosflight_firmware
  rosflight_msgs
  gazebo_ros
  )
ament_export_targets(rosflight_sil_pluginTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  rosflight_firmware
  rosflight_msgs
  gazebo_ros
)

install(
  TARGETS rosflight_sil_plugin
  EXPORT rosflight_sil_pluginTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY xacro
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
