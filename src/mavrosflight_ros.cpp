/**
 * \file mavrosflight_ros.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/serial_exception.h>
#include <string>
#include <stdint.h>

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

  mavrosflight_->register_param_value_callback(boost::bind(&MavrosflightROS::paramCallback, this, _1, _2, _3));
  mavrosflight_->register_heartbeat_callback(boost::bind(&MavrosflightROS::heartbeatCallback, this));
  mavrosflight_->register_imu_callback(boost::bind(&MavrosflightROS::imuCallback, this, _1, _2, _3, _4, _5, _6));

  mavrosflight_->send_param_request_list(1);
}

MavrosflightROS::~MavrosflightROS()
{
  delete mavrosflight_;
}

void MavrosflightROS::paramCallback(char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN], float param_value, MAV_PARAM_TYPE param_type)
{
  bool have_uint(false);
  uint32_t uint_value;

  bool have_int(false);
  int32_t int_value;

  bool have_float(false);

  switch (param_type)
  {
  case MAV_PARAM_TYPE_UINT8:
    uint_value = (uint32_t) (*(uint8_t*) &param_value);
    have_uint = true;
    break;
  case MAV_PARAM_TYPE_UINT16:
    uint_value = (uint32_t) (*(uint16_t*) &param_value);
    have_uint = true;
    break;
  case MAV_PARAM_TYPE_UINT32:
    uint_value = *(uint32_t*) &param_value;
    have_uint = true;
    break;
  case MAV_PARAM_TYPE_INT8:
    int_value = (int32_t) (*(uint8_t*) &param_value);
    have_int = true;
    break;
  case MAV_PARAM_TYPE_INT16:
    int_value = (int32_t) (*(uint16_t*) &param_value);
    have_int = true;
    break;
  case MAV_PARAM_TYPE_INT32:
    int_value = *(uint32_t*) &param_value;
    have_int = true;
    break;
  case MAV_PARAM_TYPE_REAL32:
    have_float = true;
    break;
  default:
    break;
  }

  if (have_uint)
    ROS_INFO("Got parameter %s with value %u", param_id, uint_value);
  else if (have_int)
    ROS_INFO("Got parameter %s with value %d", param_id, int_value);
  else if (have_float)
    ROS_INFO("Got parameter %s with value %f", param_id, param_value);
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
