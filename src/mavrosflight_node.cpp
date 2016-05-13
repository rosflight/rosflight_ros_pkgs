/**
 * \file mavrosflight_node.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Entry point for the mavrosflight_node executable
 */

#include <ros/ros.h>
#include "mavrosflight_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mavrosflight_node");
  mavrosflight::MavrosflightROS mavrosflight_ros;
  ros::spin();
}
