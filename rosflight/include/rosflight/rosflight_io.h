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

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>

#include <std_srvs/Trigger.h>

#include <rosflight_msgs/Attitude.h>
#include <rosflight_msgs/Barometer.h>
#include <rosflight_msgs/Airspeed.h>
#include <rosflight_msgs/Command.h>
#include <rosflight_msgs/OutputRaw.h>
#include <rosflight_msgs/RCRaw.h>
#include <rosflight_msgs/Status.h>
#include <rosflight_msgs/Error.h>

#include <rosflight_msgs/ParamFile.h>
#include <rosflight_msgs/ParamGet.h>
#include <rosflight_msgs/ParamSet.h>

#include <rosflight/mavrosflight/mavrosflight.h>
#include <rosflight/mavrosflight/mavlink_comm.h>
#include <rosflight/mavrosflight/mavlink_listener_interface.h>
#include <rosflight/mavrosflight/param_listener_interface.h>

#include <geometry_msgs/Quaternion.h>

namespace rosflight_io
{

class rosflightIO :
  public mavrosflight::MavlinkListenerInterface,
  public mavrosflight::ParamListenerInterface
{
public:
  rosflightIO();
  ~rosflightIO();

  virtual void handle_mavlink_message(const mavlink_message_t &msg);

  virtual void on_new_param_received(std::string name, double value);
  virtual void on_param_value_updated(std::string name, double value);
  virtual void on_params_saved_change(bool unsaved_changes);

  static constexpr float HEARTBEAT_PERIOD = 1; //Time between heartbeat messages

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
  void handle_named_value_int_msg(const mavlink_message_t &msg);
  void handle_named_value_float_msg(const mavlink_message_t &msg);
  void handle_named_command_struct_msg(const mavlink_message_t &msg);
  void handle_small_range_msg(const mavlink_message_t &msg);
  void handle_version_msg(const mavlink_message_t &msg);
  void handle_hard_error_msg(const mavlink_message_t &msg);

  // ROS message callbacks
  void commandCallback(rosflight_msgs::Command::ConstPtr msg);
  void attitudeCorrectionCallback(geometry_msgs::Quaternion::ConstPtr msg);

  // ROS service callbacks
  bool paramGetSrvCallback(rosflight_msgs::ParamGet::Request &req, rosflight_msgs::ParamGet::Response &res);
  bool paramSetSrvCallback(rosflight_msgs::ParamSet::Request &req, rosflight_msgs::ParamSet::Response &res);
  bool paramWriteSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool paramSaveToFileCallback(rosflight_msgs::ParamFile::Request &req, rosflight_msgs::ParamFile::Response &res);
  bool paramLoadFromFileCallback(rosflight_msgs::ParamFile::Request &req, rosflight_msgs::ParamFile::Response &res);
  bool calibrateImuBiasSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool calibrateRCTrimSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool calibrateBaroSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool calibrateAirspeedSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool rebootSrvCallback(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response &res);
  bool rebootToBootloaderSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  // timer callbacks
  void paramTimerCallback(const ros::TimerEvent &e);
  void versionTimerCallback(const ros::TimerEvent &e);
  void heartbeatTimerCallback(const ros::TimerEvent &e);

  // helpers
  void request_version();
  void send_heartbeat();
  void check_error_code(uint8_t current, uint8_t previous, ROSFLIGHT_ERROR_CODE code, std::string name);

  template<class T> inline T saturate(T value, T min, T max)
  {
    return value < min ? min : (value > max ? max : value);
  }


  ros::NodeHandle nh_;

  ros::Subscriber command_sub_;
  ros::Subscriber attitude_sub_;

  ros::Publisher unsaved_params_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher imu_temp_pub_;
  ros::Publisher output_raw_pub_;
  ros::Publisher rc_raw_pub_;
  ros::Publisher diff_pressure_pub_;
  ros::Publisher temperature_pub_;
  ros::Publisher baro_pub_;
  ros::Publisher sonar_pub_;
  ros::Publisher mag_pub_;
  ros::Publisher attitude_pub_;
  ros::Publisher euler_pub_;
  ros::Publisher status_pub_;
  ros::Publisher version_pub_;
  ros::Publisher lidar_pub_;
  ros::Publisher error_pub_;
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

  geometry_msgs::Quaternion attitude_quat_;
  mavlink_rosflight_status_t prev_status_;

  std::string frame_id_;

  mavrosflight::MavlinkComm *mavlink_comm_;
  mavrosflight::MavROSflight *mavrosflight_;
};

} // namespace rosflight_io

#endif // ROSFLIGHT_IO_MAVROSFLIGHT_ROS_H
