/**
 * \file fcu_io.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/serial_exception.h>
#include <string>
#include <stdint.h>

#include "fcu_io.h"

namespace fcu_io
{

fcuIO::fcuIO()
{
  ros::NodeHandle nh;
  imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
  param_request_list_srv_ = nh.advertiseService("param_request_list", &fcuIO::paramRequestListSrvCallback, this);
  param_set_srv_ = nh.advertiseService("param_set", &fcuIO::paramSetSrvCallback, this);

  ros::NodeHandle nh_private("~");
  std::string port = nh_private.param<std::string>("port", "/dev/ttyUSB0");
  int baud_rate = nh_private.param<int>("baud_rate", 115200);

  try
  {
    mavrosflight_ = new mavrosflight::MavROSflight(port, baud_rate);
  }
  catch (mavrosflight::SerialException e)
  {
    ROS_FATAL("%s", e.what());
    ros::shutdown();
  }

  mavrosflight_->register_param_value_callback(boost::bind(&fcuIO::paramCallback, this, _1, _2, _3));
  mavrosflight_->register_heartbeat_callback(boost::bind(&fcuIO::heartbeatCallback, this));
  mavrosflight_->register_imu_callback(boost::bind(&fcuIO::imuCallback, this, _1, _2, _3, _4, _5, _6));

  mavrosflight_->send_param_request_list(1);
}

fcuIO::~fcuIO()
{
  delete mavrosflight_;
}

bool fcuIO::paramRequestListSrvCallback(fcu_io::ParamRequestList::Request &req, fcu_io::ParamRequestList::Response &res)
{
  mavrosflight_->send_param_request_list(1);
  return true;
}

bool fcuIO::paramSetSrvCallback(fcu_io::ParamSet::Request &req, fcu_io::ParamSet::Response &res)
{
  switch (req.param_type)
  {
  case MAV_PARAM_TYPE_INT32:
    mavrosflight_->send_param_set(1, MAV_COMP_ID_ALL, req.param_id.c_str(), req.integer_value);
    return true;
  default:
    ROS_ERROR("Currently only params of type int32 are supported");
    return false;
  }

}

void fcuIO::paramCallback(char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN], float param_value, MAV_PARAM_TYPE param_type)
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

void fcuIO::heartbeatCallback()
{
  ROS_INFO_ONCE("Got HEARTBEAT, connected.");
}

void fcuIO::imuCallback(double xacc, double yacc, double zacc, double xgyro, double ygyro, double zgyro)
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

} // namespace fcu_io
