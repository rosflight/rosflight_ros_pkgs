/**
 * \file mavrosflight_ros.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/serial_exception.h>
#include <string>

#include "mavrosflight_ros.h"

namespace mavrosflight
{

MavrosflightROS::MavrosflightROS()
{
  ros::NodeHandle nh;
  imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 1);

  ros::NodeHandle nh_private("~");
  std::string port = nh_private.param<std::string>("port", "/dev/ttyUSB0");
  int baud_rate = nh_private.param<int>("baud_rate", 115200);

  try
  {
    mavrosflight_ = new MavROSflight(port, baud_rate);
  }
  catch (SerialException e)
  {
    ROS_FATAL("%s", e.what());
    ros::shutdown();
  }

  mavrosflight_->register_heartbeat_callback(boost::bind(&MavrosflightROS::heartbeatCallback, this));
  mavrosflight_->register_imu_callback(boost::bind(&MavrosflightROS::imuCallback, this, _1, _2, _3, _4, _5, _6));
}

MavrosflightROS::~MavrosflightROS()
{
  delete mavrosflight_;
}

void MavrosflightROS::heartbeatCallback()
{
  ROS_INFO_ONCE("Got HEARTBEAT, connected.");
}

void MavrosflightROS::imuCallback(double xacc, double yacc, double zacc, double xgyro, double ygyro, double zgyro)
{
  sensor_msgs::Imu msg;

  msg.linear_acceleration.x = xacc;
  msg.linear_acceleration.y = yacc;
  msg.linear_acceleration.z = zacc;

  msg.angular_velocity.x = xgyro;
  msg.angular_velocity.y = ygyro;
  msg.angular_velocity.z = zgyro;

  imu_pub_.publish(msg);
}

} // namespace mavrosflight
