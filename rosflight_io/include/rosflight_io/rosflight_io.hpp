/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
 * Copyright (c) 2025 Ian Reid, BYU MAGICC Lab.
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
 * @file rosflight_io.h
 * @author Daniel Koch <daniel.koch\@byu.edu>
 * @author Brandon Sutherland <brandonsutherland2\@gmail.com>
 * @author Ian Reid <ian.young.reid\@gmail.com>
 */
#ifndef ROSFLIGHT_IO_MAVROSFLIGHT_ROS_H
#define ROSFLIGHT_IO_MAVROSFLIGHT_ROS_H

#include <set>
#include <string>
#include <unordered_set>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <random>
#include <algorithm>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
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
#include <rosflight_msgs/msg/output_raw.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>
#include <rosflight_msgs/msg/status.hpp>

#include <rosflight_msgs/srv/param_file.hpp>
#include <rosflight_msgs/srv/param_get.hpp>
#include <rosflight_msgs/srv/param_set.hpp>

#include <rosflight_io/mavrosflight/mavlink_comm.hpp>
#include <rosflight_io/mavrosflight/mavlink_listener_interface.hpp>
#include <rosflight_io/mavrosflight/mavrosflight.hpp>
#include <rosflight_io/mavrosflight/param_listener_interface.hpp>

#include "magnetometer_calibration.hpp"

namespace rosflight_io
{
/**
 * @class ROSflightIO
 * @brief ROS code for rosflight_io node.
 *
 * This class contains all of the ROS code for the rosflight_io node. It uses the MAVROSflight
 * library to communicate with the firmware (which handles all serial communication), with
 * ROSflightIO serving as the "ROS" layer on top of MAVROSflight. MAVROSflight uses MAVLink to
 * serialize and deserialize messages between itself and the firmware, which serves as the message
 * "format".
 */
class ROSflightIO : public rclcpp::Node,
                    public mavrosflight::MavlinkListenerInterface,
                    public mavrosflight::ParamListenerInterface
{
public:
  /**
   * @brief Default constructor for ROSflightIO.
   *
   * Initializes many ROS services, subscriptions, publishers, parameters, timers, and topics,
   * as well as everything required for MAVROSflight.
   *
   * Needs to be initialized as shared pointer and then passed to rclcpp::spin() like so:
   * @code
   * rclcpp::spin(std::make_shared<rosflight_io::ROSflightIO>());
   * @endcode
   */
  ROSflightIO();
  /**
   * @brief Default de-constructor for ROSflightIO.
   *
   * Tears down everything instantiated by ROSflightIO that won't be torn down automatically.
   */
  ~ROSflightIO() override;

  /**
   * @brief Handles all MAVLink messages.
   *
   * This function handles any/all MAVLink messages received from the firmware.
   * It calls the specific private handle function needed for each message type, doing
   * nothing with the message itself.
   *
   * @param msg Mavlink message to be handled.
   */
  void handle_mavlink_message(const mavlink_message_t & msg) override;

  /**
   * @brief Callback for when new parameters are received from firmware.
   *
   * This function is a callback for whenever MAVROSflight receives new parameters. The parameters
   * can be set by either the firmware or ROS.
   *
   * Currently just prints a ROS message.
   *
   * @param name Name of parameter.
   * @param value Value of parameter.
   */
  void on_new_param_received(std::string name, double value) override;
  /**
   * @brief Callback for when existing parameters are changed.
   *
   * This function is a callback for whenever MAVROSflight receives a parameter that already
   * exists in MAVROSflight. The parameters can be set by either the firmware or ROS.
   *
   * Currently just prints a ROS message.
   *
   * @param name Name of parameter.
   * @param value Value of parameter.
   */
  void on_param_value_updated(std::string name, double value) override;
  /**
   * @brief Callback for when saved params status changes.
   *
   * This function is a callback for when MAVROSflight detects a change in parameter save status,
   * meaning that either parameters have changed, or all parameters have been saved.
   *
   * @param unsaved_changes Status of params, true if unsaved params exist, false if otherwise.
   */
  void on_params_saved_change(bool unsaved_changes) override;

  /**
   * @brief Number of second between heartbeat messages.
   */
  static constexpr long HEARTBEAT_PERIOD = 1;
  /**
   * @brief Number of seconds between version requests.
   *
   * Defines the number of seconds between firmware version requests. Requests terminate once
   * response is received.
   */
  static constexpr long VERSION_PERIOD = 10;
  /**
   * @brief Number of seconds between parameter requests.
   *
   * This value defines the number of seconds between requests for all parameters from the firmware.
   * Requests terminate once all parameters have been received.
   */
  static constexpr long PARAMETER_PERIOD = 3;

private:
  // MAVLink message handlers
  /**
   * @brief Handles heartbeat MAVLink messages.
   * @param msg Heartbeat message.
   */
  void handle_heartbeat_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles status MAVLink messages.
   *
   * Handles all MAVLink status messages. This includes arming, failsafe, rc override,
   * ROSflight errors, and control mode.
   *
   * @param msg Status message.
   */
  void handle_status_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles command acknowledgment MAVLink messages.
   *
   * @note Command values are defined by MAVLink. (At the time of writing:
   * rosflight_io/include/rosflight_io/mavlink/v1.0/message_definitions/rosflight.xml)
   *
   * @param msg Command acknowledgment message.
   */
  void handle_command_ack_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles status text MAVLink messages.
   *
   * Text is printed out as ROS messages according to their severity.
   *
   * @param msg Status text message.
   */
  void handle_statustext_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles attitude quaternion MAVLink messages.
   *
   * Calculates Euler angles from the quaternion and publishes both as a ROS topic.
   *
   * @param msg Attitude quaternion message.
   */
  void handle_attitude_quaternion_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles IMU MAVLink messages.
   *
   * Receives MAVLink IMU message and republishes it as a ROS topic.
   *
   * @param msg IMU message.
   */
  void handle_small_imu_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles ROSflight raw servo command output MAVLink messages.
   * @param msg ROSflight output raw message.
   */
  void handle_rosflight_output_raw_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles RC raw MAVLink messages.
   *
   * Receives RC receiver PWM values from MAVLink and publishes it on "rc_raw" topic.
   *
   * @param msg RC channels raw message.
   */
  void handle_rc_channels_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles differential pressure MAVLink messages.
   *
   * Receives airspeed differential pressure from MAVLink and publishes it on "airspeed" topic.
   *
   * @param msg Differential pressure message.
   */
  void handle_diff_pressure_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles barometer MAVLink messages.
   *
   * Receives barometric pressure from MAVLink and publishes it on "baro" topic.
   *
   * @param msg Barometer message.
   */
  void handle_small_baro_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles magnetometer MAVLink messages.
   *
   * Receives magnetometer data from MAVLink and publishes it on "magnetometer" topic.
   *
   * @param msg Magnetometer message.
   */
  void handle_small_mag_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles ROSflight GNSS MAVLink messages.
   *
   * Receives GNSS data from MAVLink, and uses that to publish "gnss" topic.
   *
   * @param msg ROSflight GNSS message.
   */
  void handle_rosflight_gnss_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles rangefinder MAVLink messages.
   *
   * Receives rangefinder data from MAVLink and publishes it on "sonar" or "lidar" topic, depending
   * on the sensor type.
   *
   * @param msg Range message.
   */
  void handle_small_range_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles version MAVLink messages.
   *
   * Receives firmware version from MAVLink and publishes it on "version" topic. Also cancels future
   * requests for firmware version.
   *
   * @param msg Version message.
   */
  void handle_version_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles hard error MAVLink messages.
   *
   * When hard faults occur, Receives the fault data from MAVLink and publishes it as both a ROS
   * error message and on the "rosflight_errors" topic.
   *
   * @param msg Hard error message.
   */
  void handle_hard_error_msg(const mavlink_message_t & msg);
  /**
   * @brief Handles battery status MAVLink messages.
   *
   * Receives battery voltage and current from MAVLink and publishes it on "battery" topic.
   *
   * @param msg Battery status message.
   */
  void handle_battery_status_msg(const mavlink_message_t & msg);

  void handle_offboard_control_msg(const mavlink_message_t & msg);
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr serial_delay_pub_;
  int64_t start_time_ = 0;
  int64_t sec_divisor_ = 100'000'000;
  int64_t divisor_ = 10'000;

  /**
   * @brief Parses firmware and git version strings into consistent format.
   * @param version Firmware version or ROS packages version.
   * @return Substring in consistent format.
   */
  static std::string get_major_minor_version(const std::string & version);

  // ROS message callbacks
  /**
   * @brief "command" topic subscription callback.
   *
   * This function is called anytime rosflight_io receives a message on the "command" topic.
   *
   * @param msg Populated ROSflight Command message.
   */
  void commandCallback(const rosflight_msgs::msg::Command::ConstSharedPtr & msg);
  /**
   * @brief "aux_command" topic subscription callback.
   *
   * This function is called anytime rosflight_io receives a message on the "aux_command" topic,
   * where it sends the aux command over MAVLink to the firmware.
   *
   * @param msg Populated ROSflight AuxCommand message
   */
  void auxCommandCallback(const rosflight_msgs::msg::AuxCommand::ConstSharedPtr & msg);
  /**
   * @brief "external_attitude" topic subscription callback.
   *
   * This function is called anytime rosflight_io receives a message on the "external_attitude"
   * topic, where it sends the external attitude over MAVLink to the firmware.
   *
   * @param msg Populated ROSflight Attitude message
   */
  void externalAttitudeCallback(const rosflight_msgs::msg::Attitude::ConstSharedPtr & msg);

  // ROS service callbacks
  /**
   * @brief "param_get" service callback.
   *
   * This function is called anytime the "param_get" ROS service is called. It retrieves
   * the requested param from MAVROSflight and returns it in the response.
   *
   * @param req ROSflight ParamGet service request.
   * @param res ROSflight ParamGet service response.
   * @return True
   */
  bool paramGetSrvCallback(const rosflight_msgs::srv::ParamGet::Request::SharedPtr & req,
                           const rosflight_msgs::srv::ParamGet::Response::SharedPtr & res);
  /**
   * @brief "param_set" service callback.
   *
   * This function is called anytime the "param_get" ROS service is called. It sends the param in
   * the request to MAVROSflight, which sends it to the firmware.
   *
   * @param req ROSflight ParamSet service request.
   * @param res ROSflight ParamSet service response.
   * @return True
   */
  bool paramSetSrvCallback(const rosflight_msgs::srv::ParamSet::Request::SharedPtr & req,
                           const rosflight_msgs::srv::ParamSet::Response::SharedPtr & res);
  /**
   * @brief "param_write" service callback.
   *
   * This function is called anytime the "param_write" ROS service is called. It requests a
   * param write from MAVROSflight, printing an error message if a param write is already in
   * progress.
   *
   * @param req ROS Trigger service request.
   * @param res ROS Trigger service response.
   * @return True
   */
  bool paramWriteSrvCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                             const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /**
   * @brief "param_save_to_file" service callback.
   *
   * This function is called anytime the "param_save_to_file" ROS service is called. It requests
   * a param write to file from MAVROSflight, giving it the directory in the request.
   *
   * @param req ROSflight ParamFile service request.
   * @param res ROSflight ParamFile service response.
   * @return True
   */
  bool paramSaveToFileCallback(const rosflight_msgs::srv::ParamFile::Request::SharedPtr & req,
                               const rosflight_msgs::srv::ParamFile::Response::SharedPtr & res);
  /**
   * @brief "param_load_from_file" service callback.
   *
   * This function is called anytime the "param_load_from_file" ROS service is called. It requests
   * that MAVROSflight load it's params from the file given in the ROSflight request object.
   * MAVROSflight will then load the params and sync the firmware with them.
   *
   * @param req ROSflight ParamFile service request.
   * @param res ROSflight ParamFile service response.
   * @return True
   */
  bool paramLoadFromFileCallback(const rosflight_msgs::srv::ParamFile::Request::SharedPtr & req,
                                 const rosflight_msgs::srv::ParamFile::Response::SharedPtr & res);
  /**
   * @brief "calibrate_imu" service callback.
   *
   * This function is called anytime the "calibrate_imu" ROS service is called. It signals the
   * firmware through MAVROSflight to calibrate the IMU.
   *
   * @param req ROS Trigger service request.
   * @param res ROS Trigger service response.
   * @return True
   */
  bool calibrateImuBiasSrvCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                   const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /**
   * @brief "calibrate_rc_trim" service callback.
   *
   * This function is called anytime the "calibrate_rc_trim" ROS service is called. It signals the
   * firmware through MAVROSflight to calibrate the RC trim.
   *
   * @param req ROS Trigger service request.
   * @param res ROS Trigger service response.
   * @return True
   */
  bool calibrateRCTrimSrvCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                  const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /**
   * @brief "calibrate_baro" service callback.
   *
   * This function is called anytime the "calibrate_baro" ROS service is called. It signals the
   * firmware through MAVROSflight to calibrate the baro altitude calculation.
   *
   * @param req ROS Trigger service request.
   * @param res ROS Trigger service response.
   * @return True
   */
  bool calibrateBaroSrvCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /**
   * @brief "calibrate_mag" service callback.
   *
   * This function is called anytime the "calibrate_mag" ROS service is called. It signals the
   * firmware through MAVROSflight to calibrate the magnetometer sensor.
   *
   * @param req ROS Trigger service request.
   * @param res ROS Trigger service response.
   * @return True
   */
  bool calibrateMagSrvCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /**
   * @brief "calibrate_airspeed" service callback.
   *
   * This function is called anytime the "calibrate_airspeed" ROS service is called. It signals the
   * firmware through MAVROSflight to calibrate the airspeed sensor.
   *
   * @param req ROS Trigger service request.
   * @param res ROS Trigger service response.
   * @return True
   */
  bool calibrateAirspeedSrvCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                    const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /**
   * @brief "reboot" service callback.
   *
   * This function is called anytime the "reboot" ROS service is called. It signals the firmware
   * (through MAVROSflight) to reboot itself.
   *
   * @param req ROS Trigger service request.
   * @param res ROS Trigger service response.
   * @return True
   */
  bool rebootSrvCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                         const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /**
   * @brief "reboot_to_bootloader" service callback.
   *
   * This function is called anytime the "reboot_to_bootloader" ROS service is called. It signals
   * the firmware to reboot itself to its bootloader.
   *
   * @param req ROS Trigger service request.
   * @param res ROS Trigger service response.
   * @return True
   */
  bool rebootToBootloaderSrvCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                     const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /**
   * @brief "all_params_received" service callback.
   *
   * This function is called anytime the "all_params_received" ROS service is called. It returns
   * true if all the parameters have been received from the firmware.
   *
   * @param req ROS Trigger service request.
   * @param res ROS Trigger service response.
   * @return True
   */
  bool checkIfAllParamsReceivedCallback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                        const std_srvs::srv::Trigger::Response::SharedPtr & res);

  // timer callbacks
  /**
   * @brief Callback for parameter request timer.
   *
   * This function is called repeatedly until MAVROSflight has received all parameters from the
   * firmware. It outputs ROS info/error messages with the current status of this request.
   */
  void paramTimerCallback();
  /**
   * @brief Callback for firmware version request timer.
   *
   * This function is called repeatedly until ROSflightIO receives the firmware version. It sends
   * a request for the version, but does not process the response.
   */
  void versionTimerCallback();
  /**
   * @brief Callback for the heartbeat request timer.
   *
   * This function is called repeatedly for the entire lifespan of ROSflightIO. It sends a request
   * for the firmware to send a heartbeat message.
   */
  void heartbeatTimerCallback();
  /**
   * @brief Callback for the magnetometer calibration.
   *
   * This function collects mag data and then calibrates the magnetometer.
   */
  void calibrateMag();

  // helpers
  /**
   * @brief Sends a version request to MAVROSflight.
   */
  void request_version();
  /**
   * @brief Sends a heartbeat request to MAVROSflight.
   */
  void send_heartbeat();
  /**
   * @brief Sends autopilot error/autopilot recovered error message in ROS info stream.
   *
   * This function will print out a message in the ROS info/error stream when a new error has
   * occurred or when an existing error has been resolved.
   *
   * @param current Current error code received from firmware.
   * @param previous Error code received from firmware in previous message.
   * @param code Error code to check with current function call.
   * @param name String of name to print out in ROS error/info stream.
   */
  void check_error_code(uint8_t current, uint8_t previous, ROSFLIGHT_ERROR_CODE code,
                        const std::string & name);
  /**
   * @brief Converts FCU time in MAVLink message to current ROS time.
   * @param fcu_time Chrono nanoseconds object of current FCU time.
   * @return ROS time object of current ROS time.
   */
  rclcpp::Time fcu_time_to_ros_time(std::chrono::nanoseconds fcu_time);

  /**
  * @brief Declares all the ROS2 parameters for the node.
  */
  void declare_parameters();
  /**
   * @brief loads parameters from firmware to ROS2 on boot (when all params have been received)
   * as a convenience for easy read/write access from ROS2
   */
  void load_convenience_parameters();
  /**
  * @brief Handles any parameter changes to the node.
  *
  * @param parameters Vector of rclcpp::Parameters passed by ROS2 when params are changed.
  */
  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> & parameters);

  /// "command" ROS topic subscription.
  rclcpp::Subscription<rosflight_msgs::msg::Command>::SharedPtr command_sub_;
  /// "aux_command" ROS topic subscription.
  rclcpp::Subscription<rosflight_msgs::msg::AuxCommand>::SharedPtr aux_command_sub_;
  /// "external_attitude" ROS topic subscription.
  rclcpp::Subscription<rosflight_msgs::msg::Attitude>::SharedPtr extatt_sub_;

  /// "status/unsaved_params" ROS topic publisher.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr unsaved_params_pub_;
  /// "status/rosflight_errors" ROS topic publisher.
  rclcpp::Publisher<rosflight_msgs::msg::Error>::SharedPtr error_pub_;
  /// "status/params_changed" ROS topic publisher.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr params_changed_pub_;
  /// "imu/data" ROS topic publisher.
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  /// "imu/temperature" ROS topic publisher.
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temp_pub_;
  /// "output_raw" ROS topic publisher.
  rclcpp::Publisher<rosflight_msgs::msg::OutputRaw>::SharedPtr output_raw_pub_;
  /// "rc_raw" ROS topic publisher.
  rclcpp::Publisher<rosflight_msgs::msg::RCRaw>::SharedPtr rc_raw_pub_;
  /// "airspeed" ROS topic publisher.
  rclcpp::Publisher<rosflight_msgs::msg::Airspeed>::SharedPtr diff_pressure_pub_;
  /// "baro" ROS topic publisher.
  rclcpp::Publisher<rosflight_msgs::msg::Barometer>::SharedPtr baro_pub_;
  /// "sonar" ROS topic publisher.
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub_;
  /// "gnss" ROS topic publisher.
  rclcpp::Publisher<rosflight_msgs::msg::GNSS>::SharedPtr gnss_pub_;
  /// "magnetometer" ROS topic publisher.
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  /// "attitude" ROS topic publisher.
  rclcpp::Publisher<rosflight_msgs::msg::Attitude>::SharedPtr attitude_pub_;
  /// "attitude/euler" ROS topic publisher.
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  /// "status" ROS topic publisher.
  rclcpp::Publisher<rosflight_msgs::msg::Status>::SharedPtr status_pub_;
  /// "version" ROS topic publisher.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr version_pub_;
  /// "lidar" ROS topic publisher.
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr lidar_pub_;
  /// "battery" ROS topic publisher.
  rclcpp::Publisher<rosflight_msgs::msg::BatteryStatus>::SharedPtr battery_status_pub_;

  /// "param_get" ROS service.
  rclcpp::Service<rosflight_msgs::srv::ParamGet>::SharedPtr param_get_srv_;
  /// "param_set" ROS service.
  rclcpp::Service<rosflight_msgs::srv::ParamSet>::SharedPtr param_set_srv_;
  /// "param_write" ROS service.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr param_write_srv_;
  /// "param_save_to_file" ROS service.
  rclcpp::Service<rosflight_msgs::srv::ParamFile>::SharedPtr param_save_to_file_srv_;
  /// "param_load_from_file" ROS service.
  rclcpp::Service<rosflight_msgs::srv::ParamFile>::SharedPtr param_load_from_file_srv_;
  /// "calibrate_imu" ROS service.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_calibrate_bias_srv_;
  /// "calibrate_mag" ROS service.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mag_calibrate_srv_;
  /// "calibrate_rc_trim" ROS service.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_rc_srv_;
  /// "calibrate_baro" ROS service.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_baro_srv_;
  /// "calibrate_airspeed" ROS service.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_airspeed_srv_;
  /// "reboot" ROS service.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_srv_;
  /// "reboot_to_bootloader" ROS service.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_bootloader_srv_;
  /// "all_params_received" ROS service.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr check_if_all_params_received_srv_;

  /// Handle for the ROS2 parameter callback
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  std::unordered_set<std::string> convenience_parameters_;

  /// ROS timer for param requests.
  rclcpp::TimerBase::SharedPtr param_timer_;
  /// ROS timer for firmware version requests.
  rclcpp::TimerBase::SharedPtr version_timer_;
  /// ROS timer for heartbeat requests.
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  /// ROS timer for magnetometer calibration
  rclcpp::TimerBase::SharedPtr mag_calibration_timer_;

  /// Quaternion ROS message, for passing quaternion data between functions.
  geometry_msgs::msg::Quaternion attitude_quat_;
  /// Previous firmware status, used to detect changes in status.
  mavlink_rosflight_status_t prev_status_;

  /// Frame ID string, used to include frame in published ROS message.
  std::string frame_id_;

  MagnetometerCalibrator magnetometer_calibrator_;
  
  /// Matrix of the magnetometer calibration data.
  Eigen::MatrixXd mag_calibration_data_;

  /// The current accels measured by the IMU, used for mag calibration.
  Eigen::Vector3f current_accels_;
  
  /// Indicates if the FCU is reorienting before next calibration step.
  bool reorienting_ = false;
  /// The count of how many consecutive measurements have happened in a particular orientation.
  //                               xd,xu,yd,yu,zd,zu
  std::vector<int> orientations_ = {0, 0, 0, 0, 0, 0};
  
  /// The number of times that an orientation has been checked to see if full coverage has been attained.
  int num_times_checked_;
  
  /// Whether all of the mag data from every orientation has been gathered
  bool all_orientation_data_gathered_ = false;
  
  /// The index in mag_calibration_data_ where this current orientation data starts.
  unsigned long current_orientation_index_;

  /// The best guess for the orientation of the FCU so far for mag calibration.
  unsigned long current_candidate_index_;

  std::set<int> completed_mag_orientations_;
  
  /// Flag of wether to collect calibration data.
  bool calibrate_mag_;

  /// Pointer to Mavlink communication object, used by MavROSflight.
  mavrosflight::MavlinkComm * mavlink_comm_;
  /// Pointer to MavROSflight instance, which is used for all serial communication.
  mavrosflight::MavROSflight * mavrosflight_;
};

} // namespace rosflight_io

#endif // ROSFLIGHT_IO_MAVROSFLIGHT_ROS_H
