cmake_minimum_required(VERSION 3.20...3.28)
project(rosflight_compat)

# Acknowledge variable does not need to be used since this file only defines
# a header-only library (suppresses cmake warning)
if(DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)

###############
## Libraries ##
###############

add_library(rosflight_compat_headers INTERFACE)
add_library(rosflight_compat::compat_headers ALIAS rosflight_compat_headers)
set_target_properties(rosflight_compat_headers PROPERTIES
  EXPORT_NAME compat_headers
)
target_include_directories(rosflight_compat_headers INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(rosflight_compat_headers INTERFACE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
)

#############
## Install ##
#############

# Headers for exported/public libraries
install(
  DIRECTORY include/
  DESTINATION "include/${PROJECT_NAME}"
)

# Exported/public libraries
install(
  TARGETS
    rosflight_compat_headers
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

########################
### ROS Finalization ###
########################

# Export public libraries
ament_export_targets("export_${PROJECT_NAME}" HAS_LIBRARY_TARGET)

# Export public cmake package dependencies (not targets)
ament_export_dependencies(rclcpp)

ament_package()
