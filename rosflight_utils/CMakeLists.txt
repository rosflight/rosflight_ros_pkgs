cmake_minimum_required(VERSION 3.5)
project(rosflight_utils)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(geometry_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosflight REQUIRED)
find_package(rosflight_firmware REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(gazebo_msgs REQUIRED)

include_directories(include
  ${ament_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# rosflight_utils library
add_library(rosflight_utils
  src/simple_pid.cpp
  src/turbomath.cpp
)
target_link_libraries(rosflight_utils
  ${ament_LIBRARIES}
  ${rclcpp_LIBRARIES}
  )
ament_export_targets(rosflight_utilsTargets HAS_LIBRARY_TARGET)
install(
  TARGETS rosflight_utils
  EXPORT rosflight_utilsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# joy executable
add_executable(joy src/joy.cpp)
target_link_libraries(joy
  rosflight_utils
  ${rclcpp_LIBRARIES}
  ${ament_LIBRARIES}
  )
ament_target_dependencies(joy
  gazebo_msgs
  rosflight_msgs
  sensor_msgs
  std_srvs
)

# rosflight_postprocess executable
#add_executable(rosflight_postprocess src/rosflight_postprocess.cpp)
#target_link_libraries(rosflight_postprocess
#  ${rclcpp_LIBRARIES}
#  ${ament_LIBRARIES}
#  stdc++fs
#)
#ament_target_dependencies(rosflight_postprocess
#  geometry_msgs
#  rosflight_msgs
#  sensor_msgs
#  rosflight_firmware
#  rosbag2_cpp
#)

# viz executable
add_executable(viz src/viz.cpp)
target_link_libraries(viz
  ${rclcpp_LIBRARIES}
  ${ament_LIBRARIES}
)
ament_target_dependencies(viz
  geometry_msgs
  rosflight_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

# Install header files
install(
  DIRECTORY include
  DESTINATION include
)

# Install executables
install(
  TARGETS joy viz #rosflight_postprocess
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
