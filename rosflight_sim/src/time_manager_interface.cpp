/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2025 Jacob Moore, BYU MAGICC Lab.
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
#include <chrono>

#include "rosflight_sim/time_manager_interface.hpp"

namespace rosflight_sim
{

TimeManagerInterface::TimeManagerInterface()
    : rclcpp::Node("time_manager")
{
  // Declare parameters and set up parameter change callback
  declare_parameters();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TimeManagerInterface::parameters_callback, this, std::placeholders::_1));

  // Create pause and reset service
  play_pause_srvs_ = this->create_service<std_srvs::srv::Trigger>(
    "time_manager/toggle_pause",
    std::bind(&TimeManagerInterface::toggle_pause_callback, this, std::placeholders::_1, std::placeholders::_2));

  // Create publisher
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  sim_time_pubber_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", qos);

  double default_pub_rate_us = this->get_parameter("default_pub_rate_us").as_double();
  double real_time_multiplier = this->get_parameter("real_time_multiplier").as_double();
  set_timers(default_pub_rate_us, real_time_multiplier);

  node_initialized_ = true;
}

void TimeManagerInterface::declare_parameters()
{
  // multiplier for how fast or slow simulation time goes
  this->declare_parameter("real_time_multiplier", 1.0);
  this->declare_parameter("default_pub_rate_us", 100.0);
}

rcl_interfaces::msg::SetParametersResult
TimeManagerInterface::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "Time manager is not yet initialized.";

  double default_pub_rate_us = this->get_parameter("default_pub_rate_us").as_double();
  double real_time_multiplier = this->get_parameter("real_time_multiplier").as_double();

  if (node_initialized_) {
    // Update the timer parameters
    for (auto param : parameters) {
      if (param.get_name() == "default_pub_rate_us") {
        default_pub_rate_us = param.as_double();
      } else if (param.get_name() == "real_time_multiplier") {
        real_time_multiplier = param.as_double();
      }
    }

    // Restart the timer
    sim_clock_timer_->cancel();
    set_timers(default_pub_rate_us, real_time_multiplier);

    result.successful = true;
    result.reason = "Successfully set time_manager parameter";
  }

  return result;
}

void TimeManagerInterface::set_timers(double default_pub_rate_us, double real_time_multiplier)
{
  // Note that this is specifically a wall timer, so it runs off of system time
  // and publishes according to the node parameters
  clock_duration_us_ =
    std::chrono::microseconds(static_cast<long int>(default_pub_rate_us / real_time_multiplier));
  sim_clock_timer_ = this->create_wall_timer(
    clock_duration_us_, std::bind(&TimeManagerInterface::publish_sim_time, this));
}

void TimeManagerInterface::publish_sim_time()
{
  // Update time
  update_time();

  rosgraph_msgs::msg::Clock now;
  now.clock.sec = get_seconds();
  now.clock.nanosec = get_nanoseconds();
  sim_time_pubber_->publish(now);
}

bool TimeManagerInterface::toggle_pause_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                                 const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  if (is_paused_) {
    // Unpause and restart the timer to continue publishing time information
    is_paused_ = false;

    double default_pub_rate_us = this->get_parameter("default_pub_rate_us").as_double();
    double real_time_multiplier = this->get_parameter("real_time_multiplier").as_double();
    set_timers(default_pub_rate_us, real_time_multiplier);
    RCLCPP_INFO_STREAM(this->get_logger(), "Timer is live!");
  } else {
    is_paused_ = true;

    sim_clock_timer_->cancel();
    RCLCPP_INFO_STREAM(this->get_logger(), "Timer is paused!");
  }

  return true;
}

} // namespace rosflight_sim
