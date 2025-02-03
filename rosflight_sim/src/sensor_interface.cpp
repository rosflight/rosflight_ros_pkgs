/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2024 Jacob Moore
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

#include "rosflight_sim/sensor_interface.hpp"

namespace rosflight_sim
{

SensorInterface::SensorInterface()
    : rclcpp::Node("sensors")
{
  // Declare parameters
  declare_parameters();

  // Set up the parameter callback
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&SensorInterface::parameters_callback, this, std::placeholders::_1));

  // Subscriptions
  status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>(
    "status", 1, std::bind(&SensorInterface::status_callback, this, std::placeholders::_1));
  state_sub_ = this->create_subscription<rosflight_msgs::msg::SimState>(
    "sim_state", 1, std::bind(&SensorInterface::sim_state_callback, this, std::placeholders::_1));
  wind_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "wind_truth", 1, std::bind(&SensorInterface::wind_callback, this, std::placeholders::_1));
  forces_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "forces_and_moments", 1,
    std::bind(&SensorInterface::forces_moments_callback, this, std::placeholders::_1));

  // Initialize publishers
  imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("simulated_sensors/imu/data", 1);
  imu_temperature_pub_ =
    this->create_publisher<sensor_msgs::msg::Temperature>("simulated_sensors/imu/temperature", 1);
  mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("simulated_sensors/mag", 1);
  baro_pub_ = this->create_publisher<rosflight_msgs::msg::Barometer>("simulated_sensors/baro", 1);
  gnss_pub_ = this->create_publisher<rosflight_msgs::msg::GNSS>("simulated_sensors/gnss", 1);
  gnss_full_pub_ =
    this->create_publisher<rosflight_msgs::msg::GNSSFull>("simulated_sensors/gnss_full", 1);
  diff_pressure_pub_ =
    this->create_publisher<rosflight_msgs::msg::Airspeed>("simulated_sensors/diff_pressure", 1);
  sonar_pub_ = this->create_publisher<sensor_msgs::msg::Range>("simulated_sensors/sonar", 1);
  battery_pub_ =
    this->create_publisher<rosflight_msgs::msg::BatteryStatus>("simulated_sensors/battery", 1);

  // Initialize timers with the frequencies from the parameters
  imu_update_frequency_ = this->get_parameter("imu_update_frequency").as_double();
  mag_update_frequency_ = this->get_parameter("mag_update_frequency").as_double();
  baro_update_frequency_ = this->get_parameter("baro_update_frequency").as_double();
  gnss_update_frequency_ = this->get_parameter("gnss_update_frequency").as_double();
  sonar_update_frequency_ = this->get_parameter("sonar_update_frequency").as_double();
  diff_pressure_update_frequency_ =
    this->get_parameter("diff_pressure_update_frequency").as_double();
  battery_update_frequency_ = this->get_parameter("battery_update_frequency").as_double();

  imu_timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    std::chrono::microseconds(static_cast<long long>(1.0 / imu_update_frequency_ * 1'000'000)),
    std::bind(&SensorInterface::imu_publish, this));
  mag_timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    std::chrono::microseconds(static_cast<long long>(1.0 / mag_update_frequency_ * 1'000'000)),
    std::bind(&SensorInterface::mag_publish, this));
  baro_timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    std::chrono::microseconds(static_cast<long long>(1.0 / baro_update_frequency_ * 1'000'000)),
    std::bind(&SensorInterface::baro_publish, this));
  gnss_timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    std::chrono::microseconds(static_cast<long long>(1.0 / gnss_update_frequency_ * 1'000'000)),
    std::bind(&SensorInterface::gnss_publish, this));
  diff_pressure_timer_ =
    rclcpp::create_timer(this, this->get_clock(),
                         std::chrono::microseconds(static_cast<long long>(
                           1.0 / diff_pressure_update_frequency_ * 1'000'000)),
                         std::bind(&SensorInterface::diff_pressure_publish, this));
  sonar_timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    std::chrono::microseconds(static_cast<long long>(1.0 / sonar_update_frequency_ * 1'000'000)),
    std::bind(&SensorInterface::sonar_publish, this));
  battery_timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    std::chrono::microseconds(static_cast<long long>(1.0 / battery_update_frequency_ * 1'000'000)),
    std::bind(&SensorInterface::battery_publish, this));
}

void SensorInterface::declare_parameters()
{
  // Declare all ROS2 parameters here
  this->declare_parameter("imu_update_frequency", 400.0);
  this->declare_parameter("mag_update_frequency", 50.0);
  this->declare_parameter("baro_update_frequency", 100.0);
  this->declare_parameter("gnss_update_frequency", 10.0);
  this->declare_parameter("sonar_update_frequency", 20.0);
  this->declare_parameter("diff_pressure_update_frequency", 100.0);
  this->declare_parameter("battery_update_frequency", 200.0);
}

rcl_interfaces::msg::SetParametersResult
SensorInterface::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : parameters) {
    if (param.get_name() == "imu_update_frequency") {
      reset_imu_timer(param.as_double());
      result.reason = "Success. Imu timer reset.";
    } else if (param.get_name() == "mag_update_frequency") {
      reset_mag_timer(param.as_double());
      result.reason = "Success. Mag timer reset.";
    } else if (param.get_name() == "baro_update_frequency") {
      reset_baro_timer(param.as_double());
      result.reason = "Success. baro timer reset.";
    } else if (param.get_name() == "gnss_update_frequency") {
      reset_gnss_timer(param.as_double());
      result.reason = "Success. gnss timer reset.";
    } else if (param.get_name() == "diff_pressure_update_frequency") {
      reset_diff_pressure_timer(param.as_double());
      result.reason = "Success. diff_pressure timer reset.";
    } else if (param.get_name() == "sonar_update_frequency") {
      reset_sonar_timer(param.as_double());
      result.reason = "Success. sonar timer reset.";
    } else if (param.get_name() == "battery_update_frequency") {
      reset_battery_timer(param.as_double());
      result.reason = "Success. battery_update timer reset.";
    }
  }

  return result;
}

void SensorInterface::reset_imu_timer(double frequency)
{
  if (frequency != imu_update_frequency_) {
    // Reset the timer
    imu_timer_->cancel();
    imu_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000)),
      std::bind(&SensorInterface::imu_publish, this));

    imu_update_frequency_ = frequency;
  }
}

void SensorInterface::reset_mag_timer(double frequency)
{
  if (frequency != mag_update_frequency_) {
    // Reset the timer
    mag_timer_->cancel();
    mag_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000)),
      std::bind(&SensorInterface::mag_publish, this));

    mag_update_frequency_ = frequency;
  }
}

void SensorInterface::reset_gnss_timer(double frequency)
{
  if (frequency != gnss_update_frequency_) {
    // Reset the timer
    gnss_timer_->cancel();
    gnss_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000)),
      std::bind(&SensorInterface::gnss_publish, this));

    gnss_update_frequency_ = frequency;
  }
}

void SensorInterface::reset_baro_timer(double frequency)
{
  if (frequency != baro_update_frequency_) {
    // Reset the timer
    baro_timer_->cancel();
    baro_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000)),
      std::bind(&SensorInterface::baro_publish, this));

    baro_update_frequency_ = frequency;
  }
}

void SensorInterface::reset_diff_pressure_timer(double frequency)
{
  if (frequency != diff_pressure_update_frequency_) {
    // Reset the timer
    diff_pressure_timer_->cancel();
    diff_pressure_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000)),
      std::bind(&SensorInterface::diff_pressure_publish, this));

    diff_pressure_update_frequency_ = frequency;
  }
}

void SensorInterface::reset_sonar_timer(double frequency)
{
  if (frequency != sonar_update_frequency_) {
    // Reset the timer
    sonar_timer_->cancel();
    sonar_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000)),
      std::bind(&SensorInterface::sonar_publish, this));

    sonar_update_frequency_ = frequency;
  }
}

void SensorInterface::reset_battery_timer(double frequency)
{
  if (frequency != battery_update_frequency_) {
    // Reset the timer
    battery_timer_->cancel();
    battery_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000)),
      std::bind(&SensorInterface::battery_publish, this));

    battery_update_frequency_ = frequency;
  }
}

void SensorInterface::sim_state_callback(const rosflight_msgs::msg::SimState & msg)
{
  current_state_ = msg;
}

void SensorInterface::wind_callback(const geometry_msgs::msg::Vector3Stamped & msg)
{
  current_wind_ = msg;
}

void SensorInterface::forces_moments_callback(const geometry_msgs::msg::WrenchStamped & msg)
{
  current_forces_ = msg;
}

void SensorInterface::status_callback(const rosflight_msgs::msg::Status & msg)
{
  current_status_ = msg;
}

void SensorInterface::imu_publish()
{
  sensor_msgs::msg::Imu msg = imu_update(current_state_, current_forces_);
  imu_data_pub_->publish(msg);

  sensor_msgs::msg::Temperature temp_msg = imu_temperature_update(current_state_);
  imu_temperature_pub_->publish(temp_msg);
}

void SensorInterface::mag_publish()
{
  sensor_msgs::msg::MagneticField msg = mag_update(current_state_);
  mag_pub_->publish(msg);
}

void SensorInterface::baro_publish()
{
  rosflight_msgs::msg::Barometer msg = baro_update(current_state_);
  baro_pub_->publish(msg);
}

void SensorInterface::gnss_publish()
{
  rosflight_msgs::msg::GNSS msg = gnss_update(current_state_);
  gnss_pub_->publish(msg);

  rosflight_msgs::msg::GNSSFull gnss_full_msg = gnss_full_update(current_state_);
  gnss_full_pub_->publish(gnss_full_msg);
}

void SensorInterface::diff_pressure_publish()
{
  rosflight_msgs::msg::Airspeed msg = diff_pressure_update(current_state_, current_wind_);
  diff_pressure_pub_->publish(msg);
}

void SensorInterface::sonar_publish()
{
  sensor_msgs::msg::Range msg = sonar_update(current_state_);
  sonar_pub_->publish(msg);
}

void SensorInterface::battery_publish()
{
  rosflight_msgs::msg::BatteryStatus msg = battery_update(current_state_);
  battery_pub_->publish(msg);
}

} // namespace rosflight_sim
