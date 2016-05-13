/**
 * \file mavrosflight_ros.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVROSFLIGHT_ROS_H
#define MAVROSFLIGHT_MAVROSFLIGHT_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <mavrosflight/mavrosflight.h>

namespace mavrosflight
{

class MavrosflightROS
{
public:
  MavrosflightROS();
  ~MavrosflightROS();

private:

  void heartbeatCallback();
  void imuCallback(double xacc, double yacc, double zacc, double xgyro, double ygyro, double zgyro);

  ros::Publisher imu_pub_;

  MavROSflight* mavrosflight_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_MAVROSFLIGHT_ROS_H
