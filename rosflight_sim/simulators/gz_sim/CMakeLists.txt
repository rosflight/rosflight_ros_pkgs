cmake_minimum_required(VERSION 3.8)

# Harmonic / gz-sim version of the rosflight_sim dynamics plugin
# Intended to replace the Gazebo Classic model plugin target.

if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

add_compile_options(-Wall -Wextra -Wpedantic -fPIC)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Gazebo Harmonic / gz-sim stack
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(sdformat_urdf REQUIRED)
find_package(sdformat14 REQUIRED)

add_library(gz_sim_dynamics_system SHARED
        ../../src/dynamics_interface.cpp
        src/gz_sim_dynamics_system.cpp
        src/gz_sim_dynamics.cpp)

target_include_directories(gz_sim_dynamics_system
        PRIVATE
        ../../include
        include
        gz-sim8::gz-sim8)

ament_target_dependencies(gz_sim_dynamics_system
        rclcpp
        std_srvs
        rosflight_msgs
        geometry_msgs)

target_link_libraries(gz_sim_dynamics_system
        Eigen3::Eigen
        gz-sim8::gz-sim8
        gz-plugin2::gz-plugin2
        gz-common5::gz-common5
        gz-math7::gz-math7
        gz-msgs10::gz-msgs10
        gz-transport13::gz-transport13
        sdformat14::sdformat14)
ament_target_dependencies(gz_sim_dynamics_system
        rclcpp
        std_srvs
        rosflight_msgs
        geometry_msgs)
#ament_export_targets(gazebo_dynamics_pluginTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_srvs geometry_msgs rosflight_msgs)

install(
        TARGETS gz_sim_dynamics_system
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY missions DESTINATION share/${PROJECT_NAME})
install(DIRECTORY xacro DESTINATION share/${PROJECT_NAME})
install(DIRECTORY gz_resource ../common_resource DESTINATION share/${PROJECT_NAME})
