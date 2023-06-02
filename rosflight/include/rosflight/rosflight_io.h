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
 * \file rosflight_io.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef ROSFLIGHT_IO_MAVROSFLIGHT_ROS_H
#define ROSFLIGHT_IO_MAVROSFLIGHT_ROS_H

#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <rosflight_msgs/msg/airspeed.hpp>
#include <rosflight_msgs/msg/attitude.hpp>
#include <rosflight_msgs/msg/aux_command.hpp>
#include <rosflight_msgs/msg/barometer.hpp>
#include <rosflight_msgs/msg/battery_status.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/error.hpp>
#include <rosflight_msgs/msg/gnss.hpp>
#include <rosflight_msgs/msg/gnss_full.hpp>
#include <rosflight_msgs/msg/output_raw.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>
#include <rosflight_msgs/msg/status.hpp>

#include <rosflight_msgs/srv/param_file.hpp>
#include <rosflight_msgs/srv/param_get.hpp>
#include <rosflight_msgs/srv/param_set.hpp>

#include <rosflight/mavrosflight/mavlink_comm.h>
#include <rosflight/mavrosflight/mavlink_listener_interface.h>
#include <rosflight/mavrosflight/mavrosflight.h>
#include <rosflight/mavrosflight/param_listener_interface.h>
#include <rosflight/ros_logger.h>
#include <rosflight/ros_time.h>
#include <rosflight/ros_timer.h>

namespace rosflight_io
{
class rosflightIO : public mavrosflight::MavlinkListenerInterface, public mavrosflight::ParamListenerInterface
{
public:
  rosflightIO();
  ~rosflightIO();

  virtual void handle_mavlink_message(const mavlink_message_t &msg);

  virtual void on_new_param_received(std::string name, double value);
  virtual void on_param_value_updated(std::string name, double value);
  virtual void on_params_saved_change(bool unsaved_changes);

  static constexpr float HEARTBEAT_PERIOD = 1; // Time between heartbeat messages
  static constexpr float VERSION_PERIOD = 10;  // Time between version requests
  static constexpr float PARAMETER_PERIOD = 3; // Time between parameter requests

private:
  // handle mavlink messages
  void handle_heartbeat_msg(const mavlink_message_t &msg);
  void handle_status_msg(const mavlink_message_t &msg);
  void handle_command_ack_msg(const mavlink_message_t &msg);
  void handle_statustext_msg(const mavlink_message_t &msg);
  void handle_attitude_quaternion_msg(const mavlink_message_t &msg);
  void handle_small_imu_msg(const mavlink_message_t &msg);
  void handle_rosflight_output_raw_msg(const mavlink_message_t &msg);
  void handle_rc_channels_raw_msg(const mavlink_message_t &msg);
  void handle_diff_pressure_msg(const mavlink_message_t &msg);
  void handle_small_baro_msg(const mavlink_message_t &msg);
  void handle_small_mag_msg(const mavlink_message_t &msg);
  void handle_rosflight_gnss_msg(const mavlink_message_t &msg);
  void handle_rosflight_gnss_full_msg(const mavlink_message_t &msg);
  void handle_named_value_int_msg(const mavlink_message_t &msg);
  void handle_named_value_float_msg(const mavlink_message_t &msg);
  void handle_named_command_struct_msg(const mavlink_message_t &msg);
  void handle_small_range_msg(const mavlink_message_t &msg);
  std::string get_major_minor_version(const std::string &version);
  void handle_version_msg(const mavlink_message_t &msg);
  void handle_hard_error_msg(const mavlink_message_t &msg);
  void handle_battery_status_msg(const mavlink_message_t &msg);

  // ROS message callbacks
  void commandCallback(rosflight_msgs::msg::Command::ConstSharedPtr msg);
  void auxCommandCallback(rosflight_msgs::msg::AuxCommand::ConstSharedPtr msg);
  void externalAttitudeCallback(rosflight_msgs::msg::Attitude::ConstSharedPtr msg);

  // ROS service callbacks
  bool paramGetSrvCallback(rosflight_msgs::srv::ParamGet::Request &req, rosflight_msgs::srv::ParamGet::Response &res);
  bool paramSetSrvCallback(rosflight_msgs::srv::ParamSet::Request &req, rosflight_msgs::srv::ParamSet::Response &res);
  bool paramWriteSrvCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
  bool paramSaveToFileCallback(rosflight_msgs::srv::ParamFile::Request &req, rosflight_msgs::srv::ParamFile::Response &res);
  bool paramLoadFromFileCallback(rosflight_msgs::srv::ParamFile::Request &req, rosflight_msgs::srv::ParamFile::Response &res);
  bool calibrateImuBiasSrvCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
  bool calibrateRCTrimSrvCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
  bool calibrateBaroSrvCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
  bool calibrateAirspeedSrvCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
  bool rebootSrvCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
  bool rebootToBootloaderSrvCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);

  // timer callbacks
  void paramTimerCallback();
  void versionTimerCallback();
  void heartbeatTimerCallback();

  // helpers
  void request_version();
  void send_heartbeat();
  void check_error_code(uint8_t current, uint8_t previous, ROSFLIGHT_ERROR_CODE code, std::string name);
  ros::Time fcu_time_to_ros_time(std::chrono::nanoseconds fcu_time);

  template <class T>
  inline T saturate(T value, T min, T max)
  {
    return value < min ? min : (value > max ? max : value);
  }

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<rosflight_msgs::msg::Command>::SharedPtr command_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::AuxCommand>::SharedPtr aux_command_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Attitude>::SharedPtr extatt_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr unsaved_params_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temp_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::OutputRaw>::SharedPtr output_raw_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::RCRaw>::SharedPtr rc_raw_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::Airspeed>::SharedPtr diff_pressure_pub_;
//  rclcpp::Publisher<>::SharedPtr temperature_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::Barometer>::SharedPtr baro_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::GNSS>::SharedPtr gnss_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::GNSSFull>::SharedPtr gnss_full_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_reference_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::Attitude>::SharedPtr attitude_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr version_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr lidar_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::Error>::SharedPtr error_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::BatteryStatus>::SharedPtr battery_status_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> named_value_int_pubs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> named_value_float_pubs_;
  std::map<std::string, rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr> named_command_struct_pubs_;

  rclcpp::Service<rosflight_msgs::srv::ParamGet>::SharedPtr param_get_srv_;
  rclcpp::Service<rosflight_msgs::srv::ParamSet>::SharedPtr param_set_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr param_write_srv_;
  rclcpp::Service<rosflight_msgs::srv::ParamFile>::SharedPtr param_save_to_file_srv_;
  rclcpp::Service<rosflight_msgs::srv::ParamFile>::SharedPtr param_load_from_file_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_calibrate_bias_srv_;
//  rclcpp::Service<>::SharedPtr imu_calibrate_temp_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_rc_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_baro_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_airspeed_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_bootloader_srv_;

  ros::Timer param_timer_;
  ros::Timer version_timer_;
  ros::Timer heartbeat_timer_;

  geometry_msgs::msg::Quaternion attitude_quat_;
  mavlink_rosflight_status_t prev_status_;

  std::string frame_id_;

  mavrosflight::MavlinkComm *mavlink_comm_;
  mavrosflight::MavROSflight<rosflight::ROSLogger> *mavrosflight_;

  rosflight::ROSLogger logger_;
  rosflight::ROSTimeInterface time_interface_;
  rosflight::ROSTimerProvider timer_provider_;
};

} // namespace rosflight_io

#endif // ROSFLIGHT_IO_MAVROSFLIGHT_ROS_H
