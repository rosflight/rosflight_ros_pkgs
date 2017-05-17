/**
 * \file rosflight_io_node.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Entry point for the mavrosflight_node executable
 */

#include <ros/ros.h>
#include "rosflight_io.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosflight_io_node");
  rosflight_io::rosflightIO rosflight_io;
  ros::spin();
}
