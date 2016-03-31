cmake_minimum_required(VERSION 2.8.3)
project(fcu_common)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        message_generation
)

add_message_files(
  FILES
  GPS.msg
  Command.msg
  FW_State.msg
  FW_Controller_Commands.msg
  FW_Current_Path.msg
  FW_Waypoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES fcu_common
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_definitions(-std=c++11)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(fcu_common_joy src/joy.cpp)
target_link_libraries(fcu_common_joy ${catkin_LIBRARIES})
add_dependencies(fcu_common_joy ${catkin_EXPORTED_TARGETS})

add_executable(fcu_common_gps src/gps.cpp)
target_link_libraries(fcu_common_gps ${catkin_LIBRARIES})
add_dependencies(fcu_common_gps ${catkin_EXPORTED_TARGETS})
