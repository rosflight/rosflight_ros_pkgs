/**
 * \file fcu_io_node.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Entry point for the mavrosflight_node executable
 */

#include <ros/ros.h>
#include "fcu_io.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fcu_io_node");
  fcu_io::fcuIO fcu_io;
  ros::spin();
}
