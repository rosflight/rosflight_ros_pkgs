/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
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
 * @file rosflight_io.cpp
 * @author Daniel Koch <daniel.koch\@byu.edu>
 * @author Brandon Sutherland <brandonsutherland2\@gmail.com>
 */

#include <cstdint>
#ifdef ROSFLIGHT_VERSION
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x) // Somehow, C++ requires two macros to convert a macro to a string
#define GIT_VERSION_STRING TOSTRING(ROSFLIGHT_VERSION)
#endif

#include <rosflight_io/mavrosflight/mavlink_serial.hpp>
#include <rosflight_io/mavrosflight/mavlink_udp.hpp>
#include <rosflight_io/mavrosflight/serial_exception.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h> // Swap to Eigen!
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rosflight_io/rosflight_io.hpp>
#include <iomanip>

namespace rosflight_io
{

ROSflightIO::ROSflightIO()
    : Node("rosflight_io")
    , convenience_parameters_{}
    , prev_status_()
    , magnetometer_calibrator_(1.0, 40, 0.95)
{
  command_sub_ = this->create_subscription<rosflight_msgs::msg::Command>(
    "command", 1, std::bind(&ROSflightIO::commandCallback, this, std::placeholders::_1));
  aux_command_sub_ = this->create_subscription<rosflight_msgs::msg::AuxCommand>(
    "aux_command", 1, std::bind(&ROSflightIO::auxCommandCallback, this, std::placeholders::_1));
  extatt_sub_ = this->create_subscription<rosflight_msgs::msg::Attitude>(
    "external_attitude", 1,
    std::bind(&ROSflightIO::externalAttitudeCallback, this, std::placeholders::_1));

  rclcpp::QoS qos_transient_local_1_(1);
  qos_transient_local_1_.transient_local();
  unsaved_params_pub_ =
    this->create_publisher<std_msgs::msg::Bool>("status/unsaved_params", qos_transient_local_1_);
  rclcpp::QoS qos_transient_local_5_(5); // A relatively large queue so all messages get through
  qos_transient_local_5_.transient_local();
  error_pub_ =
    this->create_publisher<rosflight_msgs::msg::Error>("status/rosflight_errors", qos_transient_local_5_);
  params_changed_pub_ =
    this->create_publisher<std_msgs::msg::Bool>("status/params_changed", 1);

  param_get_srv_ = this->create_service<rosflight_msgs::srv::ParamGet>(
    "param_get",
    std::bind(&ROSflightIO::paramGetSrvCallback, this, std::placeholders::_1,
              std::placeholders::_2));
  param_set_srv_ = this->create_service<rosflight_msgs::srv::ParamSet>(
    "param_set",
    std::bind(&ROSflightIO::paramSetSrvCallback, this, std::placeholders::_1,
              std::placeholders::_2));
  param_write_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "param_write",
    std::bind(&ROSflightIO::paramWriteSrvCallback, this, std::placeholders::_1,
              std::placeholders::_2));
  param_save_to_file_srv_ = this->create_service<rosflight_msgs::srv::ParamFile>(
    "param_save_to_file",
    std::bind(&ROSflightIO::paramSaveToFileCallback, this, std::placeholders::_1,
              std::placeholders::_2));
  param_load_from_file_srv_ = this->create_service<rosflight_msgs::srv::ParamFile>(
    "param_load_from_file",
    std::bind(&ROSflightIO::paramLoadFromFileCallback, this, std::placeholders::_1,
              std::placeholders::_2));
  imu_calibrate_bias_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "calibrate_imu",
    std::bind(&ROSflightIO::calibrateImuBiasSrvCallback, this, std::placeholders::_1,
              std::placeholders::_2));
  mag_calibrate_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "calibrate_mag",
    std::bind(&ROSflightIO::calibrateMagSrvCallback, this, std::placeholders::_1,
              std::placeholders::_2));
  calibrate_rc_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "calibrate_rc_trim",
    std::bind(&ROSflightIO::calibrateRCTrimSrvCallback, this, std::placeholders::_1,
              std::placeholders::_2));
  reboot_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "reboot",
    std::bind(&ROSflightIO::rebootSrvCallback, this, std::placeholders::_1, std::placeholders::_2));
  reboot_bootloader_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "reboot_to_bootloader",
    std::bind(&ROSflightIO::rebootToBootloaderSrvCallback, this, std::placeholders::_1,
              std::placeholders::_2));
  check_if_all_params_received_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "all_params_received",
    std::bind(&ROSflightIO::checkIfAllParamsReceivedCallback, this, std::placeholders::_1,
              std::placeholders::_2));

  declare_parameters();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ROSflightIO::parameters_callback, this, std::placeholders::_1));

  if (this->get_parameter_or("udp", false)) {
    auto bind_host = this->get_parameter_or<std::string>("bind_host", "localhost");
    auto bind_port = this->get_parameter_or<uint16_t>("bind_port", 14520);
    auto remote_host = this->get_parameter_or<std::string>("remote_host", bind_host);
    auto remote_port = this->get_parameter_or<uint16_t>("remote_port", 14525);

    RCLCPP_INFO(this->get_logger(), "Connecting over UDP to \"%s:%d\", from \"%s:%d\"",
                remote_host.c_str(), remote_port, bind_host.c_str(), bind_port);

    mavlink_comm_ = new mavrosflight::MavlinkUDP(bind_host, bind_port, remote_host, remote_port);
  } else {
    auto port = this->get_parameter_or<std::string>("port", "/dev/ttyACM0");
    int baud_rate = this->get_parameter_or<int>("baud_rate", 921600);

    RCLCPP_INFO(this->get_logger(), "Connecting to serial port \"%s\", at %d baud", port.c_str(),
                baud_rate);

    mavlink_comm_ = new mavrosflight::MavlinkSerial(port, baud_rate);
  }

  try {
    mavrosflight_ = new mavrosflight::MavROSflight(*mavlink_comm_, this);
  } catch (const mavrosflight::SerialException & e) {
    RCLCPP_FATAL(this->get_logger(), "%s", e.what());
    rclcpp::shutdown();
  }

  mavrosflight_->comm.register_mavlink_listener(this);
  mavrosflight_->param.register_param_listener(this);

  // request the param list
  mavrosflight_->param.request_params();
  // TODO: Do these need to be changed to node timers instead of wall timers?
  param_timer_ =
    this->create_wall_timer(std::chrono::seconds(PARAMETER_PERIOD),
                            std::bind(&ROSflightIO::paramTimerCallback, this), nullptr);

  // request version information
  request_version();
  version_timer_ =
    this->create_wall_timer(std::chrono::seconds(VERSION_PERIOD),
                            std::bind(&ROSflightIO::versionTimerCallback, this), nullptr);

  // initialize latched "unsaved parameters" message value
  std_msgs::msg::Bool unsaved_msg;
  unsaved_msg.data = false;
  unsaved_params_pub_->publish(unsaved_msg);

  // Set up a few other random things
  frame_id_ = this->get_parameter_or<std::string>("frame_id", "world");

  prev_status_.armed = false;
  prev_status_.failsafe = false;
  prev_status_.rc_override = false;
  prev_status_.offboard = false;
  prev_status_.control_mode = OFFBOARD_CONTROL_MODE_ENUM_END;
  prev_status_.error_code = ROSFLIGHT_ERROR_NONE;

  // Start the heartbeat
  heartbeat_timer_ =
    this->create_wall_timer(std::chrono::seconds(HEARTBEAT_PERIOD),
                            std::bind(&ROSflightIO::heartbeatTimerCallback, this), nullptr);
}

void ROSflightIO::declare_parameters()
{
  this->declare_parameter("udp", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("bind_host", rclcpp::PARAMETER_STRING);
  this->declare_parameter("bind_port", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("remote_host", rclcpp::PARAMETER_STRING);
  this->declare_parameter("remote_port", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("port", rclcpp::PARAMETER_STRING);
  this->declare_parameter("baud_rate", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);

  // Convenience parameters for access to the firmware control gains
  convenience_parameters_ = {
    "PID_ROLL_RATE_I",
    "PID_ROLL_RATE_P",
    "PID_ROLL_RATE_D",
    "PID_PITCH_RATE_P",
    "PID_PITCH_RATE_I",
    "PID_PITCH_RATE_D",
    "PID_YAW_RATE_P",
    "PID_YAW_RATE_I",
    "PID_YAW_RATE_D",
    "PID_ROLL_ANG_P",
    "PID_ROLL_ANG_I",
    "PID_ROLL_ANG_D",
    "PID_PITCH_ANG_P",
    "PID_PITCH_ANG_I",
    "PID_PITCH_ANG_D"
  };
  for (const auto& param : convenience_parameters_) {
    this->declare_parameter(param, rclcpp::PARAMETER_DOUBLE);
  }
}

rcl_interfaces::msg::SetParametersResult
ROSflightIO::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    std::string name = param.get_name();
    if (convenience_parameters_.count(name) != 0) {
      double value;
      if (!mavrosflight_->param.get_param_value(name, &value)) {
        result.successful = false;
        result.reason = "Parameter " + name + " does not exist in the firmware.";
        return result;
      }

      if (abs(value - param.as_double()) < 1e-6) {
        result.reason = "Parameter is already set with that value!";
        result.successful = true;
      } else {
        auto req = std::make_shared<rosflight_msgs::srv::ParamSet::Request>();
        auto res = std::make_shared<rosflight_msgs::srv::ParamSet::Response>();
        req->value = param.as_double();
        req->name = name;

        paramSetSrvCallback(req, res);

        result.successful = res->exists;
        result.reason = "success";
      }
    }
  }

  return result;
}

void ROSflightIO::load_convenience_parameters()
{
  // Load any parameters from the firmware into the ROS2 parameters for easy read and write access.
  double value;
  for (const auto& param : convenience_parameters_) {
    if (mavrosflight_->param.get_param_value(param, &value)) {
      this->set_parameter(rclcpp::Parameter(param, value));
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to get parameter '%s' from the firmware!", param.c_str());
    }
  }
}

ROSflightIO::~ROSflightIO()
{
  delete mavrosflight_;
  delete mavlink_comm_;
}

void ROSflightIO::handle_mavlink_message(const mavlink_message_t & msg)
{
  switch (msg.msgid) {
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
      handle_rc_channels_msg(msg);
      break;
    case MAVLINK_MSG_ID_DIFF_PRESSURE:
      handle_diff_pressure_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_BARO:
      handle_small_baro_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_RANGE:
      handle_small_range_msg(msg);
      break;
    case MAVLINK_MSG_ID_ROSFLIGHT_GNSS:
      handle_rosflight_gnss_msg(msg);
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
    case MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS:
      handle_battery_status_msg(msg);
      break;
    // timing experiment
    case MAVLINK_MSG_ID_OFFBOARD_CONTROL:
      handle_offboard_control_msg(msg);
      break;
    default:
      RCLCPP_DEBUG(this->get_logger(), "rosflight_io: Got unhandled mavlink message ID %d",
                   msg.msgid);
      break;
  }
}

void ROSflightIO::handle_offboard_control_msg(const mavlink_message_t & msg)
{
  // This function is for a timing experiment. Usually we don't get offboard command messages back.
  if (serial_delay_pub_ == nullptr) {
    serial_delay_pub_ = this->create_publisher<std_msgs::msg::Int64>("serial_time_delay", 10);
  }

  mavlink_offboard_control_t off;
  mavlink_msg_offboard_control_decode(&msg, &off);

  std_msgs::msg::Int64 out;
  int64_t curr_time = this->get_clock()->now().nanoseconds() % sec_divisor_;

  // int64_t msec_part = curr_time / divisor_;
  // int64_t frac_ms_part = curr_time % divisor_;

  int64_t off_msec = static_cast<int64_t>(std::roundf(off.Fx)) * divisor_;
  int64_t off_frac = static_cast<int64_t>(std::roundf(off.Fy));
  off_msec = off_msec + off_frac;

  out.data = curr_time - off_msec;
  if (out.data < 0) {
    out.data += sec_divisor_;
  }

  // float y = static_cast<float>(msec_part);
  // float z = static_cast<float>(frac_ms_part);
  //
  // out.data = static_cast<int64_t>((y - off.Fy)*divisor_ + (z - off.Fz));
  // std::cout << "curr_time: " << curr_time << std::endl;
  // std::cout << "y        : " << std::fixed << std::roundf(y)*divisor_ << std::endl;
  // std::cout << "z        : " << std::fixed << (z) << std::endl;
  // std::cout << "out      : " << out.data << std::endl;
  // if (out.data < 0) {
  //   out.data = static_cast<int64_t>((y - off.Fy)*divisor_ + (z - off.Fz));
  //   std::cout << "******************************************************************" << std::endl;
  // }

  serial_delay_pub_->publish(out);
}

void ROSflightIO::on_new_param_received(std::string name, double value)
{
  RCLCPP_DEBUG(this->get_logger(), "Got parameter %s with value %g", name.c_str(), value);
}

void ROSflightIO::on_param_value_updated(std::string name, double value)
{
  RCLCPP_INFO(this->get_logger(), "Parameter %s has new value %g", name.c_str(), value);

  if (mavrosflight_->param.got_all_params()) {
    // Send message that params have changed
    std_msgs::msg::Bool params_changed;
    params_changed.data = true;
    params_changed_pub_->publish(params_changed);
  }

  // Updated the ROS2 parameter if applicable. This keeps the ROS2 parameters in sync.
  if (convenience_parameters_.count(name) != 0) {
    this->set_parameter(rclcpp::Parameter(name, value));
  }
}

void ROSflightIO::on_params_saved_change(bool unsaved_changes)
{
  std_msgs::msg::Bool msg;
  msg.data = unsaved_changes;
  unsaved_params_pub_->publish(msg);

  if (unsaved_changes) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1,
                         "There are unsaved changes to onboard parameters");
  } else {
    RCLCPP_INFO(this->get_logger(), "Onboard parameters have been saved");
  }
}

void ROSflightIO::handle_heartbeat_msg(const mavlink_message_t & msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Got HEARTBEAT, connected.");
}

void ROSflightIO::handle_status_msg(const mavlink_message_t & msg)
{
  mavlink_rosflight_status_t status_msg;
  mavlink_msg_rosflight_status_decode(&msg, &status_msg);

  // armed state check
  if (prev_status_.armed != status_msg.armed) {
    if (status_msg.armed)
      RCLCPP_WARN(this->get_logger(), "Autopilot ARMED");
    else
      RCLCPP_WARN(this->get_logger(), "Autopilot DISARMED");
  }

  // failsafe check
  if (prev_status_.failsafe != status_msg.failsafe) {
    if (status_msg.failsafe)
      RCLCPP_ERROR(this->get_logger(), "Autopilot FAILSAFE");
    else
      RCLCPP_INFO(this->get_logger(), "Autopilot FAILSAFE RECOVERED");
  }

  // rc override check
  if (prev_status_.rc_override != status_msg.rc_override) {
    if (status_msg.rc_override)
      RCLCPP_WARN(this->get_logger(), "RC override active");
    else
      RCLCPP_WARN(this->get_logger(), "Returned to computer control");
  }

  // offboard control check
  if (prev_status_.offboard != status_msg.offboard) {
    if (status_msg.offboard)
      RCLCPP_WARN(this->get_logger(), "Computer control active");
    else
      RCLCPP_WARN(this->get_logger(), "Computer control lost");
  }

  // Print if got error code
  if (prev_status_.error_code != status_msg.error_code) {
    check_error_code(status_msg.error_code, prev_status_.error_code, ROSFLIGHT_ERROR_INVALID_MIXER,
                     "Invalid mixer");
    check_error_code(status_msg.error_code, prev_status_.error_code,
                     ROSFLIGHT_ERROR_IMU_NOT_RESPONDING, "IMU not responding");
    check_error_code(status_msg.error_code, prev_status_.error_code, ROSFLIGHT_ERROR_RC_LOST,
                     "RC lost");
    check_error_code(status_msg.error_code, prev_status_.error_code,
                     ROSFLIGHT_ERROR_UNHEALTHY_ESTIMATOR, "Unhealthy estimator");
    check_error_code(status_msg.error_code, prev_status_.error_code,
                     ROSFLIGHT_ERROR_TIME_GOING_BACKWARDS, "Time going backwards");
    check_error_code(status_msg.error_code, prev_status_.error_code,
                     ROSFLIGHT_ERROR_UNCALIBRATED_IMU, "Uncalibrated IMU");
    check_error_code(status_msg.error_code, prev_status_.error_code, ROSFLIGHT_ERROR_BUFFER_OVERRUN,
                     "Buffer Overrun");

    RCLCPP_DEBUG(this->get_logger(), "Autopilot ERROR 0x%02x", status_msg.error_code);
  }

  // Print if change in control mode
  if (prev_status_.control_mode != status_msg.control_mode) {
    std::string mode_string;
    switch (status_msg.control_mode) {
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
    RCLCPP_WARN_STREAM(this->get_logger(), "Autopilot now in " << mode_string << " mode");
  }

  prev_status_ = status_msg;

  // Build the status message and send it
  rosflight_msgs::msg::Status out_status;
  out_status.header.stamp = this->get_clock()->now();
  out_status.armed = status_msg.armed;
  out_status.failsafe = status_msg.failsafe;
  out_status.rc_override = status_msg.rc_override;
  out_status.offboard = status_msg.offboard;
  out_status.control_mode = status_msg.control_mode;
  out_status.error_code = status_msg.error_code;
  out_status.num_errors = status_msg.num_errors;
  out_status.loop_time_us = status_msg.loop_time_us;
  if (status_pub_ == nullptr) {
    status_pub_ = this->create_publisher<rosflight_msgs::msg::Status>("status", 1);
  }
  status_pub_->publish(out_status);
}

void ROSflightIO::handle_command_ack_msg(const mavlink_message_t & msg)
{
  mavlink_rosflight_cmd_ack_t ack;
  mavlink_msg_rosflight_cmd_ack_decode(&msg, &ack);

  if (ack.success == ROSFLIGHT_CMD_SUCCESS) {
    RCLCPP_DEBUG(this->get_logger(), "MAVLink command %d Acknowledged", ack.command);
  } else {
    RCLCPP_ERROR(this->get_logger(), "MAVLink command %d Failed", ack.command);
  }
}

void ROSflightIO::handle_statustext_msg(const mavlink_message_t & msg)
{
  mavlink_statustext_t status;
  mavlink_msg_statustext_decode(&msg, &status);

  // ensure null termination
  char c_str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
  memcpy(c_str, status.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
  c_str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = '\0';

  // Switch statement without break will execute sequentially until the next break, meaning MAV severity
  //  EMERGENCY, ALERT, CRITICAL, and ERROR will call the RCLCPP_ERROR line
  switch (status.severity) {
    case MAV_SEVERITY_EMERGENCY:
    case MAV_SEVERITY_ALERT:
    case MAV_SEVERITY_CRITICAL:
    case MAV_SEVERITY_ERROR:
      RCLCPP_ERROR(this->get_logger(), "[Autopilot]: %s", c_str);
      break;
    case MAV_SEVERITY_WARNING:
      RCLCPP_WARN(this->get_logger(), "[Autopilot]: %s", c_str);
      break;
    case MAV_SEVERITY_NOTICE:
    case MAV_SEVERITY_INFO:
      RCLCPP_INFO(this->get_logger(), "[Autopilot]: %s", c_str);
      break;
    case MAV_SEVERITY_DEBUG:
      RCLCPP_DEBUG(this->get_logger(), "[Autopilot]: %s", c_str);
      break;
  }
}

void ROSflightIO::handle_attitude_quaternion_msg(const mavlink_message_t & msg)
{
  mavlink_attitude_quaternion_t attitude;
  mavlink_msg_attitude_quaternion_decode(&msg, &attitude);

  rosflight_msgs::msg::Attitude attitude_msg;

  attitude_msg.header.stamp =
    fcu_time_to_ros_time(std::chrono::milliseconds(attitude.time_boot_ms));
  attitude_msg.attitude.w = attitude.q1;
  attitude_msg.attitude.x = attitude.q2;
  attitude_msg.attitude.y = attitude.q3;
  attitude_msg.attitude.z = attitude.q4;
  attitude_msg.angular_velocity.x = attitude.rollspeed;
  attitude_msg.angular_velocity.y = attitude.pitchspeed;
  attitude_msg.angular_velocity.z = attitude.yawspeed;

  geometry_msgs::msg::Vector3Stamped euler_msg;
  euler_msg.header.stamp = attitude_msg.header.stamp;

  tf2::Quaternion quat(attitude.q2, attitude.q3, attitude.q4, attitude.q1);
  tf2::Matrix3x3(quat).getEulerYPR(euler_msg.vector.z, euler_msg.vector.y, euler_msg.vector.x);

  // save off the quaternion for use with the IMU callback
  attitude_quat_ = tf2::toMsg(quat);

  if (attitude_pub_ == nullptr) {
    attitude_pub_ = this->create_publisher<rosflight_msgs::msg::Attitude>("attitude", 1);
  }
  if (euler_pub_ == nullptr) {
    euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("attitude/euler", 1);
  }
  attitude_pub_->publish(attitude_msg);
  euler_pub_->publish(euler_msg);
}

void ROSflightIO::handle_small_imu_msg(const mavlink_message_t & msg)
{
  mavlink_small_imu_t imu;
  mavlink_msg_small_imu_decode(&msg, &imu);

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = fcu_time_to_ros_time(std::chrono::microseconds(imu.time_boot_us));
  imu_msg.header.frame_id = frame_id_;
  imu_msg.linear_acceleration.x = imu.xacc;
  imu_msg.linear_acceleration.y = imu.yacc;
  imu_msg.linear_acceleration.z = imu.zacc;
  imu_msg.angular_velocity.x = imu.xgyro;
  imu_msg.angular_velocity.y = imu.ygyro;
  imu_msg.angular_velocity.z = imu.zgyro;
  imu_msg.orientation = attitude_quat_;

  sensor_msgs::msg::Temperature temp_msg;
  temp_msg.header.stamp = imu_msg.header.stamp;
  temp_msg.header.frame_id = frame_id_;
  temp_msg.temperature = imu.temperature;

  if (imu_pub_ == nullptr) {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
  }
  imu_pub_->publish(imu_msg);

  if (imu_temp_pub_ == nullptr) {
    imu_temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 1);
  }
  imu_temp_pub_->publish(temp_msg);

  if (calibrate_mag_) { // alpha filter the calibrated mag accel.
    Eigen::Vector3f instant_accel;
    instant_accel << imu.xacc, imu.yacc, imu.zacc;
    magnetometer_calibrator_.update_accel(instant_accel);
  }
}

void ROSflightIO::handle_rosflight_output_raw_msg(const mavlink_message_t & msg)
{
  mavlink_rosflight_output_raw_t servo;
  mavlink_msg_rosflight_output_raw_decode(&msg, &servo);

  rosflight_msgs::msg::OutputRaw out_msg;
  out_msg.header.stamp = fcu_time_to_ros_time(std::chrono::milliseconds(servo.stamp));
  for (int i = 0; i < 14; i++) {
    out_msg.values[i] = servo.values[i];
  }

  if (output_raw_pub_ == nullptr) {
    output_raw_pub_ = this->create_publisher<rosflight_msgs::msg::OutputRaw>("output_raw", 1);
  }
  output_raw_pub_->publish(out_msg);
}

void ROSflightIO::handle_rc_channels_msg(const mavlink_message_t & msg)
{
  mavlink_rc_channels_t rc;
  mavlink_msg_rc_channels_decode(&msg, &rc);

  rosflight_msgs::msg::RCRaw out_msg;
  out_msg.header.stamp = fcu_time_to_ros_time(std::chrono::milliseconds(rc.time_boot_ms));

  out_msg.values[0] = rc.chan1_raw;
  out_msg.values[1] = rc.chan2_raw;
  out_msg.values[2] = rc.chan3_raw;
  out_msg.values[3] = rc.chan4_raw;
  out_msg.values[4] = rc.chan5_raw;
  out_msg.values[5] = rc.chan6_raw;
  out_msg.values[6] = rc.chan7_raw;
  out_msg.values[7] = rc.chan8_raw;

  if (rc_raw_pub_ == nullptr) {
    rc_raw_pub_ = this->create_publisher<rosflight_msgs::msg::RCRaw>("rc_raw", 1);
  }
  rc_raw_pub_->publish(out_msg);
}

void ROSflightIO::handle_diff_pressure_msg(const mavlink_message_t & msg)
{
  mavlink_diff_pressure_t diff;
  mavlink_msg_diff_pressure_decode(&msg, &diff);

  rosflight_msgs::msg::Airspeed airspeed_msg;
  airspeed_msg.header.stamp = this->get_clock()->now();
  airspeed_msg.velocity = diff.velocity;
  airspeed_msg.differential_pressure = diff.diff_pressure;
  airspeed_msg.temperature = diff.temperature;

  if (calibrate_airspeed_srv_ == nullptr) {
    calibrate_airspeed_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "calibrate_airspeed",
      std::bind(&ROSflightIO::calibrateAirspeedSrvCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  }

  if (diff_pressure_pub_ == nullptr) {
    diff_pressure_pub_ = this->create_publisher<rosflight_msgs::msg::Airspeed>("airspeed", 1);
  }
  diff_pressure_pub_->publish(airspeed_msg);
}

void ROSflightIO::handle_small_baro_msg(const mavlink_message_t & msg)
{
  mavlink_small_baro_t baro;
  mavlink_msg_small_baro_decode(&msg, &baro);

  rosflight_msgs::msg::Barometer baro_msg;
  baro_msg.header.stamp = this->get_clock()->now();
  baro_msg.altitude = baro.altitude;
  baro_msg.pressure = baro.pressure;
  baro_msg.temperature = baro.temperature;

  // If we are getting barometer messages, then we should publish the barometer calibration service
  if (calibrate_baro_srv_ == nullptr) {
    calibrate_baro_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "calibrate_baro",
      std::bind(&ROSflightIO::calibrateBaroSrvCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  }

  if (baro_pub_ == nullptr) {
    baro_pub_ = this->create_publisher<rosflight_msgs::msg::Barometer>("baro", 1);
  }
  baro_pub_->publish(baro_msg);
}

void ROSflightIO::handle_small_mag_msg(const mavlink_message_t & msg)
{
  mavlink_small_mag_t mag;
  mavlink_msg_small_mag_decode(&msg, &mag);

  //! \todo calibration, correct units, floating point message type
  sensor_msgs::msg::MagneticField mag_msg;
  mag_msg.header.stamp = this->get_clock()->now();
  mag_msg.header.frame_id = frame_id_;

  mag_msg.magnetic_field.x = mag.xmag;
  mag_msg.magnetic_field.y = mag.ymag;
  mag_msg.magnetic_field.z = mag.zmag;

  if (mag_pub_ == nullptr) {
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("magnetometer", 1);
  }
  mag_pub_->publish(mag_msg);
  if (calibrate_mag_) {
    magnetometer_calibrator_.update_mag(mag.xmag, mag.ymag, mag.zmag);
  }
}

void ROSflightIO::handle_small_range_msg(const mavlink_message_t & msg)
{
  mavlink_small_range_t range;
  mavlink_msg_small_range_decode(&msg, &range);

  sensor_msgs::msg::Range alt_msg;
  alt_msg.header.stamp = this->get_clock()->now();
  alt_msg.max_range = range.max_range;
  alt_msg.min_range = range.min_range;
  alt_msg.range = range.range;

  switch (range.type) {
    case ROSFLIGHT_RANGE_SONAR:
      alt_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
      alt_msg.field_of_view = 1.0472; // approx 60 deg

      if (sonar_pub_ == nullptr) {
        sonar_pub_ = this->create_publisher<sensor_msgs::msg::Range>("sonar", 1);
      }
      sonar_pub_->publish(alt_msg);
      break;
    case ROSFLIGHT_RANGE_LIDAR:
      alt_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
      alt_msg.field_of_view = .0349066; // approx 2 deg

      if (lidar_pub_ == nullptr) {
        lidar_pub_ = this->create_publisher<sensor_msgs::msg::Range>("lidar", 1);
      }
      lidar_pub_->publish(alt_msg);
      break;
    default:
      break;
  }
}

rclcpp::Time ROSflightIO::fcu_time_to_ros_time(std::chrono::nanoseconds fcu_time)
{
  return rclcpp::Time(mavrosflight_->time.fcu_time_to_system_time(fcu_time).count());
}

std::string ROSflightIO::get_major_minor_version(const std::string & version)
{
  size_t start_index = 0;
  if (version[0] == 'v' || version[0] == 'V') { // Skipping the 'v' prefix
    start_index = 1;
  }
  size_t dot_index = version.find('.');         // index of the first dot
  dot_index = version.find('.', dot_index + 1); // index of the second dot
  return version.substr(start_index, dot_index - start_index);
}

void ROSflightIO::handle_version_msg(const mavlink_message_t & msg)
{
  version_timer_->cancel();

  mavlink_rosflight_version_t version;
  mavlink_msg_rosflight_version_decode(&msg, &version);

  std_msgs::msg::String version_msg;
  version_msg.data = version.version;

  if (version_pub_ == nullptr) {
    rclcpp::QoS qos_transient_local_1_(1);
    qos_transient_local_1_.transient_local();
    version_pub_ = this->create_publisher<std_msgs::msg::String>("version", qos_transient_local_1_);
  }
  version_pub_->publish(version_msg);
#ifdef GIT_VERSION_STRING // Macro so that is compiles even if git is not available
  const std::string git_version_string = GIT_VERSION_STRING;
  const std::string rosflight_major_minor_version = get_major_minor_version(git_version_string);
  const std::string firmware_version(version.version);
  const std::string firmware_major_minor_version = get_major_minor_version(firmware_version);
  if (rosflight_major_minor_version == firmware_major_minor_version) {
    RCLCPP_INFO(this->get_logger(), "ROSflight/firmware version: %s",
                firmware_major_minor_version.c_str());
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "ROSflight version does not match firmware version. Errors or missing features may result");
    RCLCPP_WARN(this->get_logger(), "ROSflight version: %s", rosflight_major_minor_version.c_str());
    RCLCPP_WARN(this->get_logger(), "Firmware version: %s", firmware_major_minor_version.c_str());
  }

#else
  RCLCPP_WARN(this->get_logger(),
              "Version checking unavailable. Firmware version may or may not be compatible with "
              "ROSflight version");
  RCLCPP_WARN(this->get_logger(), "Firmware version: %s", version.version);
#endif
}

void ROSflightIO::handle_hard_error_msg(const mavlink_message_t & msg)
{
  mavlink_rosflight_hard_error_t error;
  mavlink_msg_rosflight_hard_error_decode(&msg, &error);
  RCLCPP_ERROR(this->get_logger(),
               "Hard fault detected, with error code %u. The flight controller has rebooted.",
               error.error_code);
  RCLCPP_ERROR(this->get_logger(), "Hard fault was at: 0x%x", error.pc);
  if (error.doRearm) {
    RCLCPP_ERROR(this->get_logger(), "The firmware has rearmed itself.");
  }
  RCLCPP_ERROR(this->get_logger(), "The flight controller has rebooted %u time%s.",
               error.reset_count, error.reset_count > 1 ? "s" : "");
  rosflight_msgs::msg::Error error_msg;
  error_msg.error_message = "A firmware error has caused the flight controller to reboot.";
  error_msg.error_code = error.error_code;
  error_msg.reset_count = error.reset_count;
  error_msg.rearm = error.doRearm;
  error_msg.pc = error.pc;
  error_pub_->publish(error_msg);
}

void ROSflightIO::handle_battery_status_msg(const mavlink_message_t & msg)
{
  mavlink_rosflight_battery_status_t battery_status;
  mavlink_msg_rosflight_battery_status_decode(&msg, &battery_status);
  if (battery_status_pub_ == nullptr) {
    battery_status_pub_ = this->create_publisher<rosflight_msgs::msg::BatteryStatus>("battery", 1);
  }
  rosflight_msgs::msg::BatteryStatus battery_status_message;
  battery_status_message.voltage = battery_status.battery_voltage;
  battery_status_message.current = battery_status.battery_current;
  battery_status_message.header.stamp = this->get_clock()->now();

  battery_status_pub_->publish(battery_status_message);
}

void ROSflightIO::handle_rosflight_gnss_msg(const mavlink_message_t & msg)
{
  mavlink_rosflight_gnss_t gnss;
  mavlink_msg_rosflight_gnss_decode(&msg, &gnss);

  rclcpp::Time stamp = fcu_time_to_ros_time(std::chrono::microseconds(gnss.rosflight_timestamp));
  rosflight_msgs::msg::GNSS gnss_msg;
  gnss_msg.header.stamp = stamp;
  gnss_msg.header.frame_id = "NED";
  gnss_msg.fix_type = gnss.fix_type;
  gnss_msg.num_sat = gnss.num_sat;
  gnss_msg.lat = gnss.lat;
  gnss_msg.lon = gnss.lon;
  gnss_msg.alt = gnss.height;
  gnss_msg.vel_n = gnss.vel_n;
  gnss_msg.vel_e = gnss.vel_e;
  gnss_msg.vel_d = gnss.vel_d;
  gnss_msg.horizontal_accuracy = gnss.h_acc;
  gnss_msg.vertical_accuracy = gnss.v_acc;
  gnss_msg.speed_accuracy = gnss.s_acc;
  gnss_msg.gnss_unix_seconds = gnss.seconds;
  gnss_msg.gnss_unix_nanos = gnss.nanos;
  if (gnss_pub_ == nullptr) {
    gnss_pub_ = this->create_publisher<rosflight_msgs::msg::GNSS>("gnss", 1);
  }
  gnss_pub_->publish(gnss_msg);
}

void ROSflightIO::commandCallback(const rosflight_msgs::msg::Command::ConstSharedPtr & msg)
{
  //! \todo these are hard-coded to match right now; may want to replace with something more robust
  auto mode = (OFFBOARD_CONTROL_MODE) msg->mode;
  auto ignore = (OFFBOARD_CONTROL_IGNORE) msg->ignore;

  float Qx = msg->qx;
  float Qy = msg->qy;
  float Qz = msg->qz;
  float Fx = msg->fx;
  float Fy = msg->fy;
  float Fz = msg->fz;

  // timing experiment
  int64_t curr_time = this->get_clock()->now().nanoseconds() % sec_divisor_;

  int64_t msec_part = curr_time / divisor_;
  int64_t frac_ms_part = curr_time % divisor_;

  // Convert to float if you really want floats
  Fx = static_cast<float>(msec_part);
  Fy = static_cast<float>(frac_ms_part);

  mavlink_message_t mavlink_msg;
  mavlink_msg_offboard_control_pack(1, 50, &mavlink_msg, mode, ignore, Qx, Qy, Qz, Fx, Fy, Fz);

  mavrosflight_->comm.send_message(mavlink_msg);
}

void ROSflightIO::auxCommandCallback(const rosflight_msgs::msg::AuxCommand::ConstSharedPtr & msg)
{
  uint8_t types[14];
  float values[14];
  for (int i = 0; i < 14; i++) {
    types[i] = msg->type_array[i];
    values[i] = msg->values[i];
  }
  mavlink_message_t mavlink_msg;
  mavlink_msg_rosflight_aux_cmd_pack(1, 50, &mavlink_msg, types, values);
  mavrosflight_->comm.send_message(mavlink_msg);
}

void ROSflightIO::externalAttitudeCallback(
  const rosflight_msgs::msg::Attitude::ConstSharedPtr & msg)
{
  /// \todo Re-enable angular_velocity (or make it more clear that it is disabled)

  geometry_msgs::msg::Quaternion attitude = msg->attitude;
  // geometry_msgs::msg::Vector3 angular_velocity = msg->angular_velocity;

  mavlink_message_t mavlink_msg;
  mavlink_msg_external_attitude_pack(1, 50, &mavlink_msg, (float) attitude.w, (float) attitude.x,
                                     (float) attitude.y, (float) attitude.z);
  mavrosflight_->comm.send_message(mavlink_msg);
}

bool ROSflightIO::paramGetSrvCallback(
  const rosflight_msgs::srv::ParamGet::Request::SharedPtr & req,
  const rosflight_msgs::srv::ParamGet::Response::SharedPtr & res)
{
  res->exists = mavrosflight_->param.get_param_value(req->name, &res->value);
  return true;
}

bool ROSflightIO::paramSetSrvCallback(
  const rosflight_msgs::srv::ParamSet::Request::SharedPtr & req,
  const rosflight_msgs::srv::ParamSet::Response::SharedPtr & res)
{
  res->exists = mavrosflight_->param.set_param_value(req->name, req->value);
  return true;
}

bool ROSflightIO::paramWriteSrvCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                        const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  res->success = mavrosflight_->param.write_params();
  if (!res->success) {
    res->message = "Request rejected: write already in progress";
  }

  return true;
}

bool ROSflightIO::paramSaveToFileCallback(
  const rosflight_msgs::srv::ParamFile::Request::SharedPtr & req,
  const rosflight_msgs::srv::ParamFile::Response::SharedPtr & res)
{
  res->success = mavrosflight_->param.save_to_file(req->filename);
  return true;
}

bool ROSflightIO::paramLoadFromFileCallback(
  const rosflight_msgs::srv::ParamFile::Request::SharedPtr & req,
  const rosflight_msgs::srv::ParamFile::Response::SharedPtr & res)
{
  res->success = mavrosflight_->param.load_from_file(req->filename);
  return true;
}

bool ROSflightIO::calibrateImuBiasSrvCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_ACCEL_CALIBRATION);
  mavrosflight_->comm.send_message(msg);

  res->success = true;
  return true;
}

bool ROSflightIO::calibrateRCTrimSrvCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_RC_CALIBRATION);
  mavrosflight_->comm.send_message(msg);
  res->success = true;
  return true;
}

void ROSflightIO::paramTimerCallback()
{
  if (mavrosflight_->param.got_all_params()) {
    param_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Received all parameters");

    load_convenience_parameters();
  } else {
    mavrosflight_->param.request_params();
    RCLCPP_ERROR(this->get_logger(),
                 "Received %d of %d parameters. Requesting missing parameters...",
                 mavrosflight_->param.get_params_received(), mavrosflight_->param.get_num_params());
  }
}

void ROSflightIO::versionTimerCallback() { request_version(); }

void ROSflightIO::heartbeatTimerCallback() { send_heartbeat(); }

void ROSflightIO::request_version()
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_SEND_VERSION);
  mavrosflight_->comm.send_message(msg);
}
void ROSflightIO::send_heartbeat()
{
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(1, 50, &msg, 0, 0, 0, 0, 0);
  mavrosflight_->comm.send_message(msg);
}

void ROSflightIO::check_error_code(uint8_t current, uint8_t previous, ROSFLIGHT_ERROR_CODE code,
                                   const std::string & name)
{
  if ((current & code) != (previous & code)) {
    if (current & code) {
      RCLCPP_ERROR(this->get_logger(), "Autopilot ERROR: %s", name.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Autopilot RECOVERED ERROR: %s", name.c_str());
    }
  }
}

bool ROSflightIO::calibrateAirspeedSrvCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_AIRSPEED_CALIBRATION);
  mavrosflight_->comm.send_message(msg);
  res->success = true;
  return true;
}

bool ROSflightIO::calibrateBaroSrvCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                           const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_BARO_CALIBRATION);
  mavrosflight_->comm.send_message(msg);
  res->success = true;
  return true;
}

bool ROSflightIO::calibrateMagSrvCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res) {
 
  // Reset the mag compensation to get a clean calibration.
  mavrosflight_->param.set_param_value("MAG_X_BIAS", 0.0f);
  mavrosflight_->param.set_param_value("MAG_Y_BIAS", 0.0f);
  mavrosflight_->param.set_param_value("MAG_Z_BIAS", 0.0f);
  
  mavrosflight_->param.set_param_value("MAG_A11_COMP", 1.0f);
  mavrosflight_->param.set_param_value("MAG_A12_COMP", 0.0f);
  mavrosflight_->param.set_param_value("MAG_A13_COMP", 0.0f);
  mavrosflight_->param.set_param_value("MAG_A21_COMP", 0.0f);
  mavrosflight_->param.set_param_value("MAG_A22_COMP", 1.0f);
  mavrosflight_->param.set_param_value("MAG_A23_COMP", 0.0f);
  mavrosflight_->param.set_param_value("MAG_A31_COMP", 0.0f);
  mavrosflight_->param.set_param_value("MAG_A32_COMP", 0.0f);
  mavrosflight_->param.set_param_value("MAG_A33_COMP", 1.0f);

  float frequency = 10.0;

  auto timer_period = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000));

  // Set timer to trigger bound callback while calibrating.
  mag_calibration_timer_ = rclcpp::create_timer(this, this->get_clock(), timer_period,
                                   std::bind(&ROSflightIO::calibrateMag, this));
  
  calibrate_mag_ = true;
  res->success = true;
  res->message = "Beginning Calibration";

  // Create magnetometer calibrator 
  
  return true;
}

void ROSflightIO::calibrateMag() {

  if (magnetometer_calibrator_.calibrating()) {
    RCLCPP_INFO(this->get_logger(), magnetometer_calibrator_.feedback().c_str());
  }
  else {

    Eigen::Vector3d hard_iron_offset = magnetometer_calibrator_.get_hard_iron_offset();
    Eigen::Matrix3d soft_iron_correction = magnetometer_calibrator_.get_soft_iron_correction();

    mavrosflight_->param.set_param_value("MAG_X_BIAS", float(hard_iron_offset(0)));
    mavrosflight_->param.set_param_value("MAG_Y_BIAS", float(hard_iron_offset(1)));
    mavrosflight_->param.set_param_value("MAG_Z_BIAS", float(hard_iron_offset(2)));

    mavrosflight_->param.set_param_value("MAG_A11_COMP", float(soft_iron_correction(0,0)));
    mavrosflight_->param.set_param_value("MAG_A12_COMP", float(soft_iron_correction(0,1)));
    mavrosflight_->param.set_param_value("MAG_A13_COMP", float(soft_iron_correction(0,2)));
    mavrosflight_->param.set_param_value("MAG_A21_COMP", float(soft_iron_correction(1,0)));
    mavrosflight_->param.set_param_value("MAG_A22_COMP", float(soft_iron_correction(1,1)));
    mavrosflight_->param.set_param_value("MAG_A23_COMP", float(soft_iron_correction(1,2)));
    mavrosflight_->param.set_param_value("MAG_A31_COMP", float(soft_iron_correction(2,0)));
    mavrosflight_->param.set_param_value("MAG_A32_COMP", float(soft_iron_correction(2,1)));
    mavrosflight_->param.set_param_value("MAG_A33_COMP", float(soft_iron_correction(2,2)));

    RCLCPP_INFO(this->get_logger(), "Calibration complete.");

    mag_calibration_timer_->cancel();
    calibrate_mag_ = false;
  }
}

bool ROSflightIO::rebootSrvCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                    const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_REBOOT);
  mavrosflight_->comm.send_message(msg);
  res->success = true;
  return true;
}

bool ROSflightIO::rebootToBootloaderSrvCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 50, &msg, ROSFLIGHT_CMD_REBOOT_TO_BOOTLOADER);
  mavrosflight_->comm.send_message(msg);
  res->success = true;
  return true;
}

bool ROSflightIO::checkIfAllParamsReceivedCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  res->success = mavrosflight_->param.got_all_params();
  res->message = "Not all params received from firmware.";
  if (res->success) {
    res->message = "All params received from firmware.";
  }
  return true;
}

} // namespace rosflight_io
