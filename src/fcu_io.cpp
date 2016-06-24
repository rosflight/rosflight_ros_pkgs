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

  command_sub_ = nh.subscribe("extended_command", 1, &fcuIO::commandCallback, this);

  imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
  servo_output_raw_pub_ = nh.advertise<fcu_common::ServoOutputRaw>("servo_output_raw", 1);
  rc_raw_pub_ = nh.advertise<fcu_common::ServoOutputRaw>("rc_raw", 1);
  diff_pressure_pub_ = nh.advertise<sensor_msgs::FluidPressure>("diff_pressure", 1);
  temperature_pub_ = nh.advertise<sensor_msgs::Temperature>("temperature", 1);

  param_get_srv_ = nh.advertiseService("param_get", &fcuIO::paramGetSrvCallback, this);
  param_set_srv_ = nh.advertiseService("param_set", &fcuIO::paramSetSrvCallback, this);
  param_write_srv_ = nh.advertiseService("param_write", &fcuIO::paramWriteSrvCallback, this);

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

  mavrosflight_->serial.register_mavlink_listener(this);

  mavrosflight_->send_param_request_list(1);
}

fcuIO::~fcuIO()
{
  delete mavrosflight_;
}

void fcuIO::handle_mavlink_message(const mavlink_message_t &msg)
{
  switch (msg.msgid)
  {
  case MAVLINK_MSG_ID_HEARTBEAT:
    handle_heartbeat_msg();
    break;
  case MAVLINK_MSG_ID_PARAM_VALUE:
    handle_param_value_msg(msg);
    break;
  case MAVLINK_MSG_ID_SMALL_IMU:
    handle_small_imu_msg(msg);
    break;
  case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
    handle_servo_output_raw_msg(msg);
    break;
  case MAVLINK_MSG_ID_RC_CHANNELS:
    handle_rc_channels_raw_msg(msg);
    break;
  case MAVLINK_MSG_ID_DIFF_PRESSURE:
    handle_diff_pressure_msg(msg);
    break;
  case MAVLINK_MSG_ID_COMMAND_ACK:
    handle_command_ack_msg(msg);
    break;
  case MAVLINK_MSG_ID_NAMED_VALUE_INT:
    handle_named_value_int_msg(msg);
    break;
  case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
    handle_named_value_float_msg(msg);
    break;
  default:
    ROS_WARN_ONCE("Received unhandled mavlink message (ID = %d)", msg.msgid);
    break;
  }
}

void fcuIO::handle_heartbeat_msg()
{
  ROS_INFO_ONCE("Got HEARTBEAT, connected.");
}

void fcuIO::handle_param_value_msg(const mavlink_message_t &msg)
{
  mavlink_param_value_t param;
  mavlink_msg_param_value_decode(&msg, &param);

  bool have_uint(false);
  uint32_t uint_value;

  bool have_int(false);
  int32_t int_value;

  bool have_float(false);

  switch (param.param_type)
  {
  case MAV_PARAM_TYPE_UINT8:
    uint_value = (uint32_t) (*(uint8_t*) &param.param_value);
    have_uint = true;
    break;
  case MAV_PARAM_TYPE_UINT16:
    uint_value = (uint32_t) (*(uint16_t*) &param.param_value);
    have_uint = true;
    break;
  case MAV_PARAM_TYPE_UINT32:
    uint_value = *(uint32_t*) &param.param_value;
    have_uint = true;
    break;
  case MAV_PARAM_TYPE_INT8:
    int_value = (int32_t) (*(uint8_t*) &param.param_value);
    have_int = true;
    break;
  case MAV_PARAM_TYPE_INT16:
    int_value = (int32_t) (*(uint16_t*) &param.param_value);
    have_int = true;
    break;
  case MAV_PARAM_TYPE_INT32:
    int_value = *(uint32_t*) &param.param_value;
    have_int = true;
    break;
  case MAV_PARAM_TYPE_REAL32:
    have_float = true;
    break;
  default:
    break;
  }

  // ensure null termination on param name
  char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
  memcpy(param_id, param.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
  param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

  if (have_uint)
    ROS_INFO("Got parameter %s with value %u", param_id, uint_value);
  else if (have_int)
    ROS_INFO("Got parameter %s with value %d", param_id, int_value);
  else if (have_float)
    ROS_INFO("Got parameter %s with value %f", param_id, param.param_value);
}

void fcuIO::handle_small_imu_msg(const mavlink_message_t &msg)
{
  mavlink_small_imu_t imu;
  mavlink_msg_small_imu_decode(&msg, &imu);

  sensor_msgs::Imu out_msg;

  out_msg.header.stamp = ros::Time::now(); //! \todo time synchronization

  float accel_scale = 0.002349f;
  out_msg.linear_acceleration.x = imu.xacc * accel_scale;
  out_msg.linear_acceleration.y = imu.yacc * accel_scale;
  out_msg.linear_acceleration.z = imu.zacc * accel_scale;

  float gyro_scale = .004256f;
  out_msg.angular_velocity.x = imu.xgyro * gyro_scale;
  out_msg.angular_velocity.y = imu.ygyro * gyro_scale;
  out_msg.angular_velocity.z = imu.zgyro * gyro_scale;

  imu_pub_.publish(out_msg);
}

void fcuIO::handle_servo_output_raw_msg(const mavlink_message_t &msg)
{
  mavlink_servo_output_raw_t servo;
  mavlink_msg_servo_output_raw_decode(&msg, &servo);

  fcu_common::ServoOutputRaw out_msg;
  out_msg.header.stamp = ros::Time::now(); //! \todo time synchronization
  out_msg.port = servo.port;

  out_msg.values[0] = servo.servo1_raw;
  out_msg.values[1] = servo.servo2_raw;
  out_msg.values[2] = servo.servo3_raw;
  out_msg.values[3] = servo.servo4_raw;
  out_msg.values[4] = servo.servo5_raw;
  out_msg.values[5] = servo.servo6_raw;
  out_msg.values[6] = servo.servo7_raw;
  out_msg.values[7] = servo.servo8_raw;

  servo_output_raw_pub_.publish(out_msg);
}

void fcuIO::handle_rc_channels_raw_msg(const mavlink_message_t &msg)
{
  mavlink_rc_channels_raw_t rc;
  mavlink_msg_rc_channels_raw_decode(&msg, &rc);

  fcu_common::ServoOutputRaw out_msg;
  out_msg.header.stamp = ros::Time::now();
  out_msg.port = rc.port;

  out_msg.values[0] = rc.chan1_raw;
  out_msg.values[1] = rc.chan2_raw;
  out_msg.values[2] = rc.chan3_raw;
  out_msg.values[3] = rc.chan4_raw;
  out_msg.values[4] = rc.chan5_raw;
  out_msg.values[5] = rc.chan6_raw;
  out_msg.values[6] = rc.chan7_raw;
  out_msg.values[7] = rc.chan8_raw;

  rc_raw_pub_.publish(out_msg);
}

void fcuIO::handle_diff_pressure_msg(const mavlink_message_t &msg)
{
  mavlink_diff_pressure_t diff;
  mavlink_msg_diff_pressure_decode(&msg, &diff);

  const double P_min = -1.0f;
  const double P_max = 1.0f;
  const double PSI_to_Pa = 6894.757f;

  static int calibration_counter = 0;
  static int calibration_count = 100;
  static double _diff_pres_offset = 0.0;

  // conversion from pixhawk source code
  double temp = ((200.0f * diff.temperature) / 2047) - 50;

  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = ros::Time::now();
  temp_msg.temperature = temp;
  temperature_pub_.publish(temp_msg);

  /*
   * this equation is an inversion of the equation in the
   * pressure transfer function figure on page 4 of the datasheet
   * We negate the result so that positive differential pressures
   * are generated when the bottom port is used as the static
   * port on the pitot and top port is used as the dynamic port
   */
  double diff_press_PSI = -((diff.diff_pressure - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min);
  double diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;
  if (calibration_counter > calibration_count)
  {
    diff_press_pa_raw -= _diff_pres_offset;

    sensor_msgs::FluidPressure pressure_msg;
    pressure_msg.header.stamp = ros::Time::now();
    pressure_msg.fluid_pressure = diff_press_pa_raw;
    diff_pressure_pub_.publish(pressure_msg);
  }
  else if (calibration_counter == calibration_count)
  {
    _diff_pres_offset = _diff_pres_offset/calibration_count;
    calibration_counter++;
  }
  else
  {
    _diff_pres_offset += diff_press_pa_raw;
    calibration_counter++;
  }
}

void fcuIO::handle_command_ack_msg(const mavlink_message_t &msg)
{
  mavlink_command_ack_t ack;
  mavlink_msg_command_ack_decode(&msg, &ack);

  ROS_INFO("Acknowledged command %d with result %d", ack.command, ack.result);
}

void fcuIO::handle_named_value_int_msg(const mavlink_message_t &msg)
{
  mavlink_named_value_int_t val;
  mavlink_msg_named_value_int_decode(&msg, &val);

  std::string name(val.name);
  if (named_value_int_pubs_.find(name) == named_value_int_pubs_.end())
  {
    ros::NodeHandle nh;
    named_value_int_pubs_[name] = nh.advertise<std_msgs::Int32>("named_value/int/" + name, 1);
  }

  std_msgs::Int32 out_msg;
  out_msg.data = val.value;

  named_value_int_pubs_[name].publish(out_msg);
}

void fcuIO::handle_named_value_float_msg(const mavlink_message_t &msg)
{
  mavlink_named_value_float_t val;
  mavlink_msg_named_value_float_decode(&msg, &val);

  std::string name(val.name);
  if (named_value_float_pubs_.find(name) == named_value_float_pubs_.end())
  {
    ros::NodeHandle nh;
    named_value_float_pubs_[name] = nh.advertise<std_msgs::Float32>("named_value/float/" + name, 1);
  }

  std_msgs::Float32 out_msg;
  out_msg.data = val.value;

  named_value_float_pubs_[name].publish(out_msg);
}

void fcuIO::commandCallback(fcu_common::ExtendedCommand::ConstPtr msg)
{
  //! \todo these are hard-coded to match right now; may want to replace with something more robust
  OFFBOARD_CONTROL_MODE mode = (OFFBOARD_CONTROL_MODE) msg->mode;
  OFFBOARD_CONTROL_IGNORE ignore = (OFFBOARD_CONTROL_IGNORE) msg->ignore;

  mavrosflight_->send_command(mode, ignore, msg->value1, msg->value2, msg->value3, msg->value4);
}

bool fcuIO::paramGetSrvCallback(fcu_io::ParamGet::Request &req, fcu_io::ParamGet::Response &res)
{
  res.exists = mavrosflight_->param.get_param_value(req.name, &res.value);
  return true;
}

bool fcuIO::paramSetSrvCallback(fcu_io::ParamSet::Request &req, fcu_io::ParamSet::Response &res)
{
  res.exists = mavrosflight_->param.set_param_value(req.name, req.value);
  return true;
}

bool fcuIO::paramWriteSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = mavrosflight_->param.write_params();
  if (!res.success)
  {
    res.message = "Request rejected: write already in progress";
  }

  return true;
}

} // namespace fcu_io
