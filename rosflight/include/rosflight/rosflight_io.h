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

#include <geometry_msgs/msg/Quaternion.hpp>
#include <geometry_msgs/msg/TwistStamped.hpp>
#include <std_msgs/msg/Bool.hpp>
#include <std_msgs/msg/Float32.hpp>
#include <std_msgs/msg/Int32.hpp>
#include <std_msgs/msg/String.hpp>

#include <sensor_msgs/msg/FluidPressure.hpp>
#include <sensor_msgs/msg/Imu.hpp>
#include <sensor_msgs/msg/MagneticField.hpp>
#include <sensor_msgs/msg/NavSatFix.hpp>
#include <sensor_msgs/msg/Range.hpp>
#include <sensor_msgs/msg/Temperature.hpp>
#include <sensor_msgs/msg/TimeReference.hpp>

#include <std_srvs/srv/Trigger.hpp>

#include <rosflight_msgs/msg/airspeed.hpp>
#include <rosflight_msgs/msg/attitude.hpp>
#include <rosflight_msgs/msg/aux_command.hpp>
#include <rosflight_msgs/msg/barometer.hpp>
#include <rosflight_msgs/msg/BatteryStatus.h>
#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/error.hpp>
#include <rosflight_msgs/msg/GNSS.hpp>
#include <rosflight_msgs/msg/GNSS_full.hpp>
#include <rosflight_msgs/msg/output_raw.hpp>
#include <rosflight_msgs/msg/RC_raw.hpp>
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

  static constexpr float HEARTBEAT_PERIOD = 1; // Time between heartbeat messages, in seconds
  static constexpr float VERSION_PERIOD = 10;  // Time between version requests, in seconds
  static constexpr float PARAMETER_PERIOD = 3; // Time between parameter requests, in seconds

private:

  // the three functions to handle quaternions
  static void normalize_quat(double &w, double &x, double &y, double &z);
  static void convert_quat_to_euler(double w, double x, double y, double z, double &roll, double &pitch, double &yaw);
  static void fill_quat_message(double q1, double q2, double q3, double q4, geometry_msgs::msg::Quaternion &q);

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
  void commandCallback(rosflight_msgs::msg::Command::ConstPtr msg);
  void auxCommandCallback(rosflight_msgs::msg::AuxCommand::ConstPtr msg);
  void externalAttitudeCallback(geometry_msgs::msg::Quaternion::ConstPtr msg);

  // ROS service callbacks
  void paramGetSrvCallback(const std::shared_ptr<rosflight_msgs::srv::ParamGet::Request> req, std::shared_ptr<rosflight_msgs::srv::ParamGet::Response> res)
  void paramSetSrvCallback(const std::shared_ptr<rosflight_msgs::srv::ParamSet::Request> req, std::shared_ptr<rosflight_msgs::srv::ParamSet::Response> res)
  void paramSaveToFileCallback(const std::shared_ptr<rosflight_msgs::srv::ParamFile::Request> req, std::shared_ptr<rosflight_msgs::srv::ParamFile::Response> res)
  void paramLoadFromFileCallback(const std::shared_ptr<rosflight_msgs::srv::ParamFile::Request> req, std::shared_ptr<rosflight_msgs::srv::ParamFile::Response> res)
  void paramWriteSrvCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  void calibrateImuBiasSrvCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  void calibrateRCTrimSrvCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  void calibrateBaroSrvCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  void calibrateAirspeedSrvCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  void rebootSrvCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  void rebootToBootloaderSrvCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)

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

  std::shared_ptr<rclcpp::Node> nh_;

  ros::Subscriber command_sub_;
  ros::Subscriber aux_command_sub_;
  ros::Subscriber extatt_sub_;

  ros::Publisher unsaved_params_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher imu_temp_pub_;
  ros::Publisher output_raw_pub_;
  ros::Publisher rc_raw_pub_;
  ros::Publisher diff_pressure_pub_;
  ros::Publisher temperature_pub_;
  ros::Publisher baro_pub_;
  ros::Publisher sonar_pub_;
  ros::Publisher gnss_pub_;
  ros::Publisher gnss_full_pub_;
  ros::Publisher nav_sat_fix_pub_;
  ros::Publisher twist_stamped_pub_;
  ros::Publisher time_reference_pub_;
  ros::Publisher mag_pub_;
  ros::Publisher attitude_pub_;
  ros::Publisher euler_pub_;
  ros::Publisher status_pub_;
  ros::Publisher version_pub_;
  ros::Publisher lidar_pub_;
  ros::Publisher error_pub_;
  ros::Publisher battery_status_pub_;
  std::map<std::string, ros::Publisher> named_value_int_pubs_;
  std::map<std::string, ros::Publisher> named_value_float_pubs_;
  std::map<std::string, ros::Publisher> named_command_struct_pubs_;

  ros::ServiceServer param_get_srv_;
  ros::ServiceServer param_set_srv_;
  ros::ServiceServer param_write_srv_;
  ros::ServiceServer param_save_to_file_srv_;
  ros::ServiceServer param_load_from_file_srv_;
  ros::ServiceServer imu_calibrate_bias_srv_;
  ros::ServiceServer imu_calibrate_temp_srv_;
  ros::ServiceServer calibrate_rc_srv_;
  ros::ServiceServer calibrate_baro_srv_;
  ros::ServiceServer calibrate_airspeed_srv_;
  ros::ServiceServer reboot_srv_;
  ros::ServiceServer reboot_bootloader_srv_;

  ros::Timer param_timer_;
  ros::Timer version_timer_;
  ros::Timer heartbeat_timer_;

  // TODO: This clock is not going to work for sim time
  // It needs to come from a node handle
  rclcpp::Clock _ros_clock;

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
