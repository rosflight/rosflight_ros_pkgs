cmake_minimum_required(VERSION 3.20...3.28)
project(rosflight_gcs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# viz executable
add_executable(viz src/viz.cpp)
target_link_libraries(viz PRIVATE
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  rosflight_msgs::rosflight_msgs
  sensor_msgs::sensor_msgs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  visualization_msgs::visualization_msgs
)

# Install header files
install(
  DIRECTORY include
  DESTINATION include
)

# Install executables
install(
  TARGETS viz
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install rviz files
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
