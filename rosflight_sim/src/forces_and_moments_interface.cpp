/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
 * Copyright (c) 2025 Gabe Snow
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

#include "rosflight_sim/forces_and_moments_interface.hpp"

namespace rosflight_sim
{

ForcesAndMomentsInterface::ForcesAndMomentsInterface()
  : rclcpp::Node("forces_and_moments")
{
  // Note that we don't define the parameter callback routine here.
  // This is so that implementation-specific details can be included there.

  // Define ROS interfaces
  forces_moments_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/forces_and_moments", 1);
  truth_sub_ = this->create_subscription<rosflight_msgs::msg::SimState>(
    "sim/truth_state", 1, std::bind(&ForcesAndMomentsInterface::state_callback, this, std::placeholders::_1));
  wind_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "sim/wind_truth", 1, std::bind(&ForcesAndMomentsInterface::wind_callback, this, std::placeholders::_1));
  firware_out_sub_ = this->create_subscription<rosflight_msgs::msg::PwmOutput>(
    "sim/pwm_output", 1, std::bind(&ForcesAndMomentsInterface::firmware_output_callback, this, std::placeholders::_1));
}

void ForcesAndMomentsInterface::state_callback(const rosflight_msgs::msg::SimState & msg)
{
  current_state_ = msg;
}

void ForcesAndMomentsInterface::wind_callback(const geometry_msgs::msg::Vector3Stamped & msg)
{
  current_wind_ = msg;
}

void ForcesAndMomentsInterface::firmware_output_callback(const rosflight_msgs::msg::PwmOutput & msg)
{
  // Update the forces and moments
  geometry_msgs::msg::WrenchStamped forces_moments = update_forces_and_torques(current_state_, current_wind_, msg.values);

  // Publish forces and moments
  forces_moments_pub_->publish(forces_moments);
}

} // namespace rosflight_sim
