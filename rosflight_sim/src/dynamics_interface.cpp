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

#include "rosflight_sim/dynamics_interface.hpp"

namespace rosflight_sim
{

DynamicsInterface::DynamicsInterface()
    : rclcpp::Node("dynamics")
{
  // Declare parameters and set up the callback for changing parameters
  declare_parameters();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DynamicsInterface::parameters_callback, this, std::placeholders::_1));

  // Initialize publishers, subscribers, and services
  truth_state_pub_ = this->create_publisher<rosflight_msgs::msg::SimState>("sim/truth_state", 1);
  wind_truth_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("sim/truth_wind", 1);
  forces_moments_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "forces_and_moments", 1,
    std::bind(&DynamicsInterface::forces_callback, this, std::placeholders::_1));

  // Initialize service server
  set_sim_state_srvs_ = this->create_service<rosflight_msgs::srv::SetSimState>(
    "dynamics/set_sim_state",
    std::bind(&DynamicsInterface::set_sim_state_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void DynamicsInterface::declare_parameters()
{
  // Declare parameters for the dynamics node here
}

rcl_interfaces::msg::SetParametersResult
DynamicsInterface::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

void DynamicsInterface::forces_callback(const geometry_msgs::msg::WrenchStamped & msg)
{
  apply_forces_and_torques(msg);

  // Compute truth state
  // TODO: This should probably be on a different timer than the output from the board. 
  // A zero-order-hold would be more realistic. Perhaps we put a timer in the forces and moments that 
  // controls how fast propagation happens.
  rosflight_msgs::msg::SimState truth = compute_truth();
  truth.header.stamp = this->get_clock()->now();
  truth_state_pub_->publish(truth);

  // Compute wind truth
  // Wind should be in the inertial frame
  geometry_msgs::msg::Vector3Stamped wind_truth = compute_wind_truth();
  wind_truth.header.stamp = this->get_clock()->now();
  wind_truth_pub_->publish(wind_truth);
}

bool DynamicsInterface::set_sim_state_callback(const rosflight_msgs::srv::SetSimState::Request::SharedPtr req,
                                               const rosflight_msgs::srv::SetSimState::Response::SharedPtr res)
{
  res->success = set_sim_state(req->state);
  res->message = res->success ? "Sim state set." : "Sim state not set!";
  return true;
}

} // namespace rosflight_sim
