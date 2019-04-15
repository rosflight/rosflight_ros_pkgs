/*
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file rosflight_io.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <rosflight/mavrosflight/mavlink_serial.h>
#include <rosflight/mavrosflight/mavlink_udp.h>
#include <rosflight/mavrosflight/serial_exception.h>
#include <string>
#include <stdint.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>

#include <rosflight/rosflight_io.h>

namespace rosflight_io
{
rosflightIO::rosflightIO()
{
  command_sub_ = nh_.subscribe("command", 1, &rosflightIO::commandCallback, this);
  attitude_sub_ = nh_.subscribe("attitude_correction", 1, &rosflightIO::attitudeCorrectionCallback, this);

  unsaved_params_pub_ = nh_.advertise<std_msgs::Bool>("unsaved_params", 1, true);
  error_pub_ = nh_.advertise<rosflight_msgs::Error>("rosflight_errors",5,true); // A relatively large queue so all messages get through

  param_get_srv_ = nh_.advertiseService("param_get", &rosflightIO::paramGetSrvCallback, this);
  param_set_srv_ = nh_.advertiseService("param_set", &rosflightIO::paramSetSrvCallback, this);
  param_write_srv_ = nh_.advertiseService("param_write", &rosflightIO::paramWriteSrvCallback, this);
  param_save_to_file_srv_ = nh_.advertiseService("param_save_to_file", &rosflightIO::paramSaveToFileCallback, this);
  param_load_from_file_srv_ = nh_.advertiseService("param_load_from_file", &rosflightIO::paramLoadFromFileCallback, this);
  imu_calibrate_bias_srv_ = nh_.advertiseService("calibrate_imu", &rosflightIO::calibrateImuBiasSrvCallback, this);
  calibrate_rc_srv_ = nh_.advertiseService("calibrate_rc_trim", &rosflightIO::calibrateRCTrimSrvCallback, this);
  reboot_srv_ = nh_.advertiseService("reboot", &rosflightIO::rebootSrvCallback, this);
  reboot_bootloader_srv_ = nh_.advertiseService("reboot_to_bootloader", &rosflightIO::rebootToBootloaderSrvCallback, this);
  
  ros::NodeHandle nh_private("~");

  if (nh_private.param<bool>("udp", false))
  {
    std::string bind_host = nh_private.param<std::string>("bind_host", "localhost");
    uint16_t bind_port = (uint16_t) nh_private.param<int>("bind_port", 14520);
    std::string remote_host = nh_private.param<std::string>("remote_host", bind_host);
    uint16_t remote_port = (uint16_t) nh_private.param<int>("remote_port", 14525);

    ROS_INFO("Connecting over UDP to \"%s:%d\", from \"%s:%d\"", remote_host.c_str(), remote_port, bind_host.c_str(), bind_port);

    mavlink_comm_ = new mavrosflight::MavlinkUDP(bind_host, bind_port, remote_host, remote_port);
  }
  else
  {
    std::string port = nh_private.param<std::string>("port", "/dev/ttyUSB0");
    int baud_rate = nh_private.param<int>("baud_rate", 921600);

    ROS_INFO("Connecting to serial port \"%s\", at %d baud", port.c_str(), baud_rate);

    mavlink_comm_ = new mavrosflight::MavlinkSerial(port, baud_rate);
  }

  try
  {
    mavlink_comm_->open(); //! \todo move this into the MavROSflight constructor
    mavrosflight_ = new mavrosflight::MavROSflight(*mavlink_comm_);
  }
  catch (mavrosflight::SerialException e)
  {
    ROS_FATAL("%s", e.what());
    ros::shutdown();
  }

  mavrosflight_->comm.register_mavlink_listener(this);
  mavrosflight_->param.register_param_listener(this);

  // request the param list
  mavrosflight_->param.request_params();
  param_timer_ = nh_.createTimer(ros::Duration(1.0), &rosflightIO::paramTimerCallback, this);

  // request version information
  request_version();
  version_timer_ = nh_.createTimer(ros::Duration(1.0), &rosflightIO::versionTimerCallback, this);

  // initialize latched "unsaved parameters" message value
  std_msgs::Bool unsaved_msg;
  unsaved_msg.data = false;
  unsaved_params_pub_.publish(unsaved_msg);

  // Set up a few other random things
  frame_id_ = nh_private.param<std::string>("frame_id", "world");

  prev_status_.armed = false;
  prev_status_.failsafe = false;
  prev_status_.rc_override = false;
  prev_status_.offboard = false;
  prev_status_.control_mode = OFFBOARD_CONTROL_MODE_ENUM_END;
  prev_status_.error_code = ROSFLIGHT_ERROR_NONE;

  //Start the heartbeat
  heartbeat_timer_ = nh_.createTimer(ros::Duration(HEARTBEAT_PERIOD), &rosflightIO::heartbeatTimerCallback, this);
}

rosflightIO::~rosflightIO()
{
  delete mavrosflight_;
  delete mavlink_comm_;
}

void rosflightIO::handle_mavlink_message(const mavlink_message_t &msg)
{
  switch (msg.msgid)
  {
    case MAVLINK_MSG_ID_HEARTBEAT:
      handle_heartbeat_msg(msg);
      break;
    case MAVLINK_MSG_ID_ROSFLIGHT_STATUS:
      handle_status_msg(msg);
      break;
    case MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK:
      handle_command_ack_msg(msg);
      break;
    case MAVLINK_MSG_ID_STATUSTEXT:
      handle_statustext_msg(msg);
      break;
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
      handle_attitude_quaternion_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_IMU:
      handle_small_imu_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_MAG:
      handle_small_mag_msg(msg);
      break;
    case MAVLINK_MSG_ID_ROSFLIGHT_OUTPUT_RAW:
      handle_rosflight_output_raw_msg(msg);
      break;
    case MAVLINK_MSG_ID_RC_CHANNELS:
      handle_rc_channels_raw_msg(msg);
      break;
    case MAVLINK_MSG_ID_DIFF_PRESSURE:
      handle_diff_pressure_msg(msg);
      break;
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
      handle_named_value_int_msg(msg);
      break;
    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
      handle_named_value_float_msg(msg);
      break;
    case MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT:
      handle_named_command_struct_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_BARO:
      handle_small_baro_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_RANGE:
      handle_small_range_msg(msg);
      break;
    case MAVLINK_MSG_ID_ROSFLIGHT_VERSION:
      handle_version_msg(msg);
      break;
    case MAVLINK_MSG_ID_PARAM_VALUE:
    case MAVLINK_MSG_ID_TIMESYNC:
      // silently ignore (handled elsewhere)
      break;
    case MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR:
      handle_hard_error_msg(msg);
      break;
    default:
      ROS_DEBUG("rosflight_io: Got unhandled mavlink message ID %d", msg.msgid);
      break;
  }
}

void rosflightIO::on_new_param_received(std::string name, double value)
{
  ROS_DEBUG("Got parameter %s with value %g", name.c_str(), value);
}

void rosflightIO::on_param_value_updated(std::string name, double value)
{
  ROS_INFO("Parameter %s has new value %g", name.c_str(), value);
}

void rosflightIO::on_params_saved_change(bool unsaved_changes)
{
  std_msgs::Bool msg;
  msg.data = unsaved_changes;
  unsaved_params_pub_.publish(msg);

  if (unsaved_changes)
  {
    ROS_WARN_THROTTLE(1,"There are unsaved changes to onboard parameters");
  }
  else
  {
    ROS_INFO("Onboard parameters have been saved");
  }
}

void rosflightIO::handle_heartbeat_msg(const mavlink_message_t &msg)
{
  ROS_INFO_ONCE("Got HEARTBEAT, connected.");
}

void rosflightIO::handle_status_msg(const mavlink_message_t &msg)
{
  mavlink_rosflight_status_t status_msg;
  mavlink_msg_rosflight_status_decode(&msg, &status_msg);

  // armed state check
  if (prev_status_.armed != status_msg.armed)
  {
    if (status_msg.armed)
      ROS_WARN("Autopilot ARMED");
    else
      ROS_WARN("Autopilot DISARMED");
  }

  // failsafe check
  if (prev_status_.failsafe != status_msg.failsafe)
  {
    if (status_msg.failsafe)
      ROS_ERROR("Autopilot FAILSAFE");
    else
      ROS_INFO("Autopilot FAILSAFE RECOVERED");
  }

  // rc override check
  if (prev_status_.rc_override != status_msg.rc_override)
  {
    if (status_msg.rc_override)
      ROS_WARN("RC override active");
    else
      ROS_WARN("Returned to computer control");
  }

  // offboard control check
  if (prev_status_.offboard != status_msg.offboard)
  {
    if (status_msg.offboard)
      ROS_WARN("Computer control active");
    else
      ROS_WARN("Computer control lost");
  }

  // Print if got error code
  if (prev_status_.error_code != status_msg.error_code)
  {
    check_error_code(status_msg.error_code, prev_status_.error_code, ROSFLIGHT_ERROR_INVALID_MIXER, "Invalid mixer");
    check_error_code(status_msg.error_code, prev_status_.error_code, ROSFLIGHT_ERROR_IMU_NOT_RESPONDING, "IMU not responding");
    check_error_code(status_msg.error_code, prev_status_.error_code, ROSFLIGHT_ERROR_RC_LOST, "RC lost");
    check_error_code(status_msg.error_code, prev_status_.error_code, ROSFLIGHT_ERROR_UNHEALTHY_ESTIMATOR, "Unhealthy estimator");
    check_error_code(status_msg.error_code, prev_status_.error_code, ROSFLIGHT_ERROR_TIME_GOING_BACKWARDS, "Time going backwards");
    check_error_code(status_msg.error_code, prev_status_.error_code, ROSFLIGHT_ERROR_UNCALIBRATED_IMU, "Uncalibrated IMU");
    check_error_code(status_msg.error_code, prev_status_.error_code, ROSFLIGHT_ERROR_BUFFER_OVERRUN, "Buffer Overrun");

    ROS_DEBUG("Autopilot ERROR 0x%02x", status_msg.error_code);
  }

  // Print if change in control mode
  if (prev_status_.control_mode != status_msg.control_mode)
  {
    std::string mode_string;
    switch (status_msg.control_mode)
    {
      case MODE_PASS_THROUGH:
        mode_string = "PASS_THROUGH";
        break;
      case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
        mode_string = "RATE";
        break;
      case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
        mode_string = "ANGLE";
        break;
      default:
        mode_string = "UNKNOWN";
    }
    ROS_WARN_STREAM("Autopilot now in " << mode_string << " mode");
  }

  prev_status_ = status_msg;

  // Build the status message and send it
  rosflight_msgs::Status out_status;
  out_status.header.stamp = ros::Time::now();
  out_status.armed = status_msg.armed;
  out_status.failsafe = status_msg.failsafe;
  out_status.rc_override = status_msg.rc_override;
  out_status.offboard = status_msg.offboard;
  out_status.control_mode = status_msg.control_mode;
  out_status.error_code = status_msg.error_code;
  out_status.num_errors = status_msg.num_errors;
  out_status.loop_time_us = status_msg.loop_time_us;
  if (status_pub_.getTopic().empty())
  {
    status_pub_ = nh_.advertise<rosflight_msgs::Status>("status", 1);
  }
  status_pub_.publish(out_status);
}

void rosflightIO::handle_command_ack_msg(const mavlink_message_t &msg)
{
  mavlink_rosflight_cmd_ack_t ack;
  mavlink_msg_rosflight_cmd_ack_decode(&msg, &ack);

  if (ack.success == ROSFLIGHT_CMD_SUCCESS)
  {
    ROS_DEBUG("MAVLink command %d Acknowledged", ack.command);
  }
  else
  {
    ROS_ERROR("MAVLink command %d Failed", ack.command);
  }
}

void rosflightIO::handle_statustext_msg(const mavlink_message_t &msg)
{
  mavlink_statustext_t status;
  mavlink_msg_statustext_decode(&msg, &status);

  // ensure null termination
  char c_str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
  memcpy(c_str, status.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
  c_str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = '\0';

  switch (status.severity)
  {
    case MAV_SEVERITY_EMERGENCY:
    case MAV_SEVERITY_ALERT:
    case MAV_SEVERITY_CRITICAL:
    case MAV_SEVERITY_ERROR:
      ROS_ERROR("[Autopilot]: %s", c_str);
      break;
    case MAV_SEVERITY_WARNING:
      ROS_WARN("[Autopilot]: %s", c_str);
      break;
    case MAV_SEVERITY_NOTICE:
    case MAV_SEVERITY_INFO:
      ROS_INFO("[Autopilot]: %s", c_str);
      break;
    case MAV_SEVERITY_DEBUG:
      ROS_DEBUG("[Autopilot]: %s", c_str);
      break;
  }
}

void rosflightIO::handle_attitude_quaternion_msg(const mavlink_message_t &msg)
{
  mavlink_attitude_quaternion_t attitude;
  mavlink_msg_attitude_quaternion_decode(&msg, &attitude);

  rosflight_msgs::Attitude attitude_msg;
  attitude_msg.header.stamp = mavrosflight_->time.get_ros_time_ms(attitude.time_boot_ms);
  attitude_msg.attitude.w = attitude.q1;
  attitude_msg.attitude.x = attitude.q2;
  attitude_msg.attitude.y = attitude.q3;
  attitude_msg.attitude.z = attitude.q4;
  attitude_msg.angular_velocity.x = attitude.rollspeed;
  attitude_msg.angular_velocity.y = attitude.pitchspeed;
  attitude_msg.angular_velocity.z = attitude.yawspeed;

  geometry_msgs::Vector3Stamped euler_msg;
  euler_msg.header.stamp = attitude_msg.header.stamp;

  tf::Quaternion quat(attitude.q2, attitude.q3, attitude.q4, attitude.q1);
  tf::Matrix3x3(quat).getEulerYPR(euler_msg.vector.z, euler_msg.vector.y, euler_msg.vector.x);

  // save off the quaternion for use with the IMU callback
  tf::quaternionTFToMsg(quat, attitude_quat_);

  if (attitude_pub_.getTopic().empty())
  {
    attitude_pub_ = nh_.advertise<rosflight_msgs::Attitude>("attitude", 1);
  }
  if (euler_pub_.getTopic().empty())
  {
    euler_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("attitude/euler", 1);
  }
  attitude_pub_.publish(attitude_msg);
  euler_pub_.publish(euler_msg);
}

void rosflightIO::handle_small_imu_msg(const mavlink_message_t &msg)
{
  mavlink_small_imu_t imu;
  mavlink_msg_small_imu_decode(&msg, &imu);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = mavrosflight_->time.get_ros_time_us(imu.time_boot_us);
  imu_msg.header.frame_id = frame_id_;
  imu_msg.linear_acceleration.x = imu.xacc;
  imu_msg.linear_acceleration.y = imu.yacc;
  imu_msg.linear_acceleration.z = imu.zacc;
  imu_msg.angular_velocity.x = imu.xgyro;
  imu_msg.angular_velocity.y = imu.ygyro;
  imu_msg.angular_velocity.z = imu.zgyro;
  imu_msg.orientation = attitude_quat_;

  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = imu_msg.header.stamp;
  temp_msg.header.frame_id = frame_id_;
  temp_msg.temperature = imu.temperature;

  if (imu_pub_.getTopic().empty())
  {
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
  }
  imu_pub_.publish(imu_msg);

  if (imu_temp_pub_.getTopic().empty())
  {
    imu_temp_pub_ = nh_.advertise<sensor_msgs::Temperature>("imu/temperature", 1);
  }
  imu_temp_pub_.publish(temp_msg);
}

void rosflightIO::handle_rosflight_output_raw_msg(const mavlink_message_t &msg)
{
  mavlink_rosflight_output_raw_t servo;
  mavlink_msg_rosflight_output_raw_decode(&msg, &servo);

  rosflight_msgs::OutputRaw out_msg;
  out_msg.header.stamp = mavrosflight_->time.get_ros_time_us(servo.stamp);
  for (int i = 0; i < 8; i++)
  {
    out_msg.values[i] = servo.values[i];
  }

  if (output_raw_pub_.getTopic().empty())
  {
    output_raw_pub_ = nh_.advertise<rosflight_msgs::OutputRaw>("output_raw", 1);
  }
  output_raw_pub_.publish(out_msg);
}

void rosflightIO::handle_rc_channels_raw_msg(const mavlink_message_t &msg)
{
  mavlink_rc_channels_raw_t rc;
  mavlink_msg_rc_channels_raw_decode(&msg, &rc);

  rosflight_msgs::RCRaw out_msg;
  out_msg.header.stamp = mavrosflight_->time.get_ros_time_ms(rc.time_boot_ms);

  out_msg.values[0] = rc.chan1_raw;
  out_msg.values[1] = rc.chan2_raw;
  out_msg.values[2] = rc.chan3_raw;
  out_msg.values[3] = rc.chan4_raw;
  out_msg.values[4] = rc.chan5_raw;
  out_msg.values[5] = rc.chan6_raw;
  out_msg.values[6] = rc.chan7_raw;
  out_msg.values[7] = rc.chan8_raw;

  if (rc_raw_pub_.getTopic().empty())
  {
    rc_raw_pub_ = nh_.advertise<rosflight_msgs::RCRaw>("rc_raw", 1);
  }
  rc_raw_pub_.publish(out_msg);
}

void rosflightIO::handle_diff_pressure_msg(const mavlink_message_t &msg)
{
  mavlink_diff_pressure_t diff;
  mavlink_msg_diff_pressure_decode(&msg, &diff);

  rosflight_msgs::Airspeed airspeed_msg;
  airspeed_msg.header.stamp = ros::Time::now();
  airspeed_msg.velocity = diff.velocity;
  airspeed_msg.differential_pressure = diff.diff_pressure;
  airspeed_msg.temperature = diff.temperature;

  if(calibrate_airspeed_srv_.getService().empty())
  {
    calibrate_airspeed_srv_ = nh_.advertiseService("calibrate_airspeed", &rosflightIO::calibrateAirspeedSrvCallback, this);
  }

  if (diff_pressure_pub_.getTopic().empty())
  {
    diff_pressure_pub_ = nh_.advertise<rosflight_msgs::Airspeed>("airspeed", 1);
  }
  diff_pressure_pub_.publish(airspeed_msg);
}

void rosflightIO::handle_named_value_int_msg(const mavlink_message_t &msg)
{
  mavlink_named_value_int_t val;
  mavlink_msg_named_value_int_decode(&msg, &val);

  // ensure null termination of name
  char c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN + 1];
  memcpy(c_name, val.name, MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN);
  c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN] = '\0';
  std::string name(c_name);

  if (named_value_int_pubs_.find(name) == named_value_int_pubs_.end())
  {
    ros::NodeHandle nh;
    named_value_int_pubs_[name] = nh.advertise<std_msgs::Int32>("named_value/int/" + name, 1);
  }

  std_msgs::Int32 out_msg;
  out_msg.data = val.value;

  named_value_int_pubs_[name].publish(out_msg);
}

void rosflightIO::handle_named_value_float_msg(const mavlink_message_t &msg)
{
  mavlink_named_value_float_t val;
  mavlink_msg_named_value_float_decode(&msg, &val);

  // ensure null termination of name
  char c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN + 1];
  memcpy(c_name, val.name, MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN);
  c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN] = '\0';
  std::string name(c_name);

  if (named_value_float_pubs_.find(name) == named_value_float_pubs_.end())
  {
    ros::NodeHandle nh;
    named_value_float_pubs_[name] = nh.advertise<std_msgs::Float32>("named_value/float/" + name, 1);
  }

  std_msgs::Float32 out_msg;
  out_msg.data = val.value;

  named_value_float_pubs_[name].publish(out_msg);
}

void rosflightIO::handle_named_command_struct_msg(const mavlink_message_t &msg)
{
  mavlink_named_command_struct_t command;
  mavlink_msg_named_command_struct_decode(&msg, &command);

  // ensure null termination of name
  char c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN + 1];
  memcpy(c_name, command.name, MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN);
  c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN] = '\0';
  std::string name(c_name);

  if (named_command_struct_pubs_.find(name) == named_command_struct_pubs_.end())
  {
    ros::NodeHandle nh;
    named_command_struct_pubs_[name] = nh.advertise<rosflight_msgs::Command>("named_value/command_struct/" + name, 1);
  }

  rosflight_msgs::Command command_msg;
  if (command.type == MODE_PASS_THROUGH)
    command_msg.mode = rosflight_msgs::Command::MODE_PASS_THROUGH;
  else if (command.type == MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE)
    command_msg.mode = rosflight_msgs::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  else if (command.type == MODE_ROLL_PITCH_YAWRATE_THROTTLE)
    command_msg.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  else if (command.type == MODE_ROLL_PITCH_YAWRATE_ALTITUDE)
    command_msg.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_ALTITUDE;

  command_msg.ignore = command.ignore;
  command_msg.x = command.x;
  command_msg.y = command.y;
  command_msg.z = command.z;
  command_msg.F = command.F;
  named_command_struct_pubs_[name].publish(command_msg);
}

void rosflightIO::handle_small_baro_msg(const mavlink_message_t &msg)
{
  mavlink_small_baro_t baro;
  mavlink_msg_small_baro_decode(&msg, &baro);

  rosflight_msgs::Barometer baro_msg;
  baro_msg.header.stamp = ros::Time::now();
  baro_msg.altitude = baro.altitude;
  baro_msg.pressure = baro.pressure;
  baro_msg.temperature = baro.temperature;

  // If we are getting barometer messages, then we should publish the barometer calibration service
  if(calibrate_baro_srv_.getService().empty())
  {
    calibrate_baro_srv_ = nh_.advertiseService("calibrate_baro", &rosflightIO::calibrateBaroSrvCallback, this);
  }

  if (baro_pub_.getTopic().empty())
  {
    baro_pub_ = nh_.advertise<rosflight_msgs::Barometer>("baro", 1);
  }
  baro_pub_.publish(baro_msg);
}

void rosflightIO::handle_small_mag_msg(const mavlink_message_t &msg)
{
  mavlink_small_mag_t mag;
  mavlink_msg_small_mag_decode(&msg, &mag);

  //! \todo calibration, correct units, floating point message type
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.stamp = ros::Time::now();//mavrosflight_->time.get_ros_time_us(mag.time_boot_us);
  mag_msg.header.frame_id = frame_id_;

  mag_msg.magnetic_field.x = mag.xmag;
  mag_msg.magnetic_field.y = mag.ymag;
  mag_msg.magnetic_field.z = mag.zmag;

  if (mag_pub_.getTopic().empty())
  {
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("magnetometer", 1);
  }
  mag_pub_.publish(mag_msg);
}

void rosflightIO::handle_small_range_msg(const mavlink_message_t &msg)
{
  mavlink_small_range_t range;
  mavlink_msg_small_range_decode(&msg, &range);

  sensor_msgs::Range alt_msg;
  alt_msg.header.stamp = ros::Time::now();
  alt_msg.max_range = range.max_range;
  alt_msg.min_range = range.min_range;
  alt_msg.range = range.range;

  switch(range.type) {
    case ROSFLIGHT_RANGE_SONAR:
      alt_msg.radiation_type  = sensor_msgs::Range::ULTRASOUND;
      alt_msg.field_of_view   = 1.0472;  // approx 60 deg

      if (sonar_pub_.getTopic().empty())
      {
        sonar_pub_ = nh_.advertise<sensor_msgs::Range>("sonar", 1);
      }
      sonar_pub_.publish(alt_msg);
      break;
    case ROSFLIGHT_RANGE_LIDAR:
      alt_msg.radiation_type  = sensor_msgs::Range::INFRARED;
      alt_msg.field_of_view   = .0349066; //approx 2 deg
      
      if (lidar_pub_.getTopic().empty())
      {
        lidar_pub_ = nh_.advertise<sensor_msgs::Range>("lidar", 1);
      }
      lidar_pub_.publish(alt_msg);
      break;
    default:
      break;
  }

}

void rosflightIO::handle_version_msg(const mavlink_message_t &msg)
{
  version_timer_.stop();

  mavlink_rosflight_version_t version;
  mavlink_msg_rosflight_version_decode(&msg, &version);

  std_msgs::String version_msg;
  version_msg.data = version.version;

  if (version_pub_.getTopic().empty())
  {
    version_pub_ = nh_.advertise<std_msgs::String>("version", 1, true);
  }
  version_pub_.publish(version_msg);

  ROS_INFO("Firmware version: %s", version.version);
}

void rosflightIO::handle_hard_error_msg(const mavlink_message_t &msg)
{
  mavlink_rosflight_hard_error_t error;
  mavlink_msg_rosflight_hard_error_decode(&msg,&error);
  ROS_ERROR("Hard fault detected, with error code %u. The flight controller has rebooted.",error.error_code);
  ROS_ERROR("Hard fault was at: 0x%x",error.pc);
  if(error.doRearm)
  {
    ROS_ERROR("The firmware has rearmed itself.");
  }
  ROS_ERROR("The flight controller has rebooted %u time%s.", error.reset_count, error.reset_count>1?"s":"");
  rosflight_msgs::Error error_msg;
  error_msg.error_message = "A firmware error has caused the flight controller to reboot.";
  error_msg.error_code = error.error_code;
  error_msg.reset_count = error.reset_count;
  error_msg.rearm = error.doRearm;
  error_msg.pc = error.pc;
  error_pub_.publish(error_msg);
}

void rosflightIO::commandCallback(rosflight_msgs::Command::ConstPtr msg)
{
  //! \todo these are hard-coded to match right now; may want to replace with something more robust
  OFFBOARD_CONTROL_MODE mode = (OFFBOARD_CONTROL_MODE)msg->mode;
  OFFBOARD_CONTROL_IGNORE ignore = (OFFBOARD_CONTROL_IGNORE)msg->ignore;

  float x = msg->x;
  float y = msg->y;
  float z = msg->z;
  float F = msg->F;

  switch (mode)
  {
    case MODE_PASS_THROUGH:
      x = saturate(x, -1.0f, 1.0f);
      y = saturate(y, -1.0f, 1.0f);
      z = saturate(z, -1.0f, 1.0f);
      F = saturate(F, 0.0f, 1.0f);
      break;
    case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
    case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
      F = saturate(F, 0.0f, 1.0f);
      break;
    case MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
      break;
  }

  mavlink_message_t mavlink_msg;
  mavlink_msg_offboard_control_pack(1, 50, &mavlink_msg, mode, ignore, x, y, z, F);
  mavrosflight_->comm.send_message(mavlink_msg);
}

void rosflightIO::attitudeCorrectionCallback(geometry_msgs::Quaternion::ConstPtr msg)
{
  mavlink_message_t mavlink_msg;
  mavlink_msg_attitude_correction_pack(1, 50, &mavlink_msg, msg->w, msg->x, msg->y, msg->z);
  mavrosflight_->comm.send_message(mavlink_msg);
}

bool rosflightIO::paramGetSrvCallback(rosflight_msgs::ParamGet::Request &req, rosflight_msgs::ParamGet::Response &res)
{
  res.exists = mavrosflight_->param.get_param_value(req.name, &res.value);
  return true;
}

bool rosflightIO::paramSetSrvCallback(rosflight_msgs::ParamSet::Request &req, rosflight_msgs::ParamSet::Response &res)
{
  res.exists = mavrosflight_->param.set_param_value(req.name, req.value);
  return true;
}

bool rosflightIO::paramWriteSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = mavrosflight_->param.write_params();
  if (!res.success)
  {
    res.message = "Request rejected: write already in progress";
  }

  return true;
}

bool rosflightIO::paramSaveToFileCallback(rosflight_msgs::ParamFile::Request &req, rosflight_msgs::ParamFile::Response &res)
{
  res.success = mavrosflight_->param.save_to_file(req.filename);
  return true;
}

bool rosflightIO::paramLoadFromFileCallback(rosflight_msgs::ParamFile::Request &req, rosflight_msgs::ParamFile::Response &res)
{
  res.success = mavrosflight_->param.load_from_file(req.filename);
  return true;
}

bool rosflightIO::calibrateImuBiasSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_ACCEL_CALIBRATION);
  mavrosflight_->comm.send_message(msg);

  res.success = true;
  return true;
}

bool rosflightIO::calibrateRCTrimSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_RC_CALIBRATION);
  mavrosflight_->comm.send_message(msg);
  res.success = true;
  return true;
}

void rosflightIO::paramTimerCallback(const ros::TimerEvent &e)
{
  if (mavrosflight_->param.got_all_params())
  {
    param_timer_.stop();
    ROS_INFO("Received all parameters");
  }
  else
  {
    mavrosflight_->param.request_params();
    ROS_ERROR("Received %d of %d parameters. Requesting missing parameters...",
             mavrosflight_->param.get_params_received(), mavrosflight_->param.get_num_params());
  }
}

void rosflightIO::versionTimerCallback(const ros::TimerEvent &e)
{
  request_version();
}

void rosflightIO::heartbeatTimerCallback(const ros::TimerEvent &e)
{
  send_heartbeat();
}

void rosflightIO::request_version()
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_SEND_VERSION);
  mavrosflight_->comm.send_message(msg);
}
void rosflightIO::send_heartbeat()
{
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(1, 50, &msg, 0,0,0,0,0);
  mavrosflight_->comm.send_message(msg);
}

void rosflightIO::check_error_code(uint8_t current, uint8_t previous, ROSFLIGHT_ERROR_CODE code, std::string name)
{
  if ((current & code) != (previous & code))
  {
    if (current & code)
      ROS_ERROR("Autopilot ERROR: %s", name.c_str());
    else
      ROS_INFO("Autopilot RECOVERED ERROR: %s", name.c_str());
  }
}

bool rosflightIO::calibrateAirspeedSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_AIRSPEED_CALIBRATION);
  mavrosflight_->comm.send_message(msg);
  res.success = true;
  return true;
}

bool rosflightIO::calibrateBaroSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_BARO_CALIBRATION);
  mavrosflight_->comm.send_message(msg);
  res.success = true;
  return true;
}

bool rosflightIO::rebootSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_REBOOT);
  mavrosflight_->comm.send_message(msg);
  res.success = true;
  return true;
}

bool rosflightIO::rebootToBootloaderSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_REBOOT_TO_BOOTLOADER);
  mavrosflight_->comm.send_message(msg);
  res.success = true;
  return true;
}

} // namespace rosflight_io
