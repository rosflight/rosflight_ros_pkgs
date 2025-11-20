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
  : rclcpp::Node("mav_forces_and_moments")
  , primary_mixing_matrix_(Eigen::Matrix<double, 6, NUM_MIXER_OUTPUTS>::Zero())
  , secondary_mixing_matrix_(Eigen::Matrix<double, 6, NUM_MIXER_OUTPUTS>::Zero())
  , mixer_header_vals_(Eigen::Matrix<int, 1, NUM_MIXER_OUTPUTS>::Zero())
{
  // Note that we don't define the parameter callback routine here.
  // This is so that implementation-specific details can be included there.
  this->declare_parameter("invert_mixing_matrix", true);

  // Define ROS interfaces
  forces_moments_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/forces_and_moments", 1);
  truth_sub_ = this->create_subscription<rosflight_msgs::msg::SimState>(
    "sim/truth_state", 1, std::bind(&ForcesAndMomentsInterface::state_callback, this, std::placeholders::_1));
  wind_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "sim/truth_wind", 1, std::bind(&ForcesAndMomentsInterface::wind_callback, this, std::placeholders::_1));
  firware_out_sub_ = this->create_subscription<rosflight_msgs::msg::PwmOutput>(
    "sim/pwm_output", 1, std::bind(&ForcesAndMomentsInterface::firmware_output_callback, this, std::placeholders::_1));

  sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // TODO:_Does this need to be a member variable?
  rclcpp::SubscriptionOptions options;
  options.callback_group = sub_cb_group_;
  params_changed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "status/params_changed", 1, std::bind(&ForcesAndMomentsInterface::params_changed_callback, this, std::placeholders::_1), options);

  // Service clients
  client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  firmware_param_get_client_ = this->create_client<rosflight_msgs::srv::ParamGet>("param_get", rmw_qos_profile_services_default, client_cb_group_);
  firmware_check_param_client_ = this->create_client<std_srvs::srv::Trigger>("all_params_received", rmw_qos_profile_services_default, client_cb_group_);

  // Request mixer values on boot
  do_initialize_parameters_ = true;
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
  if (do_initialize_parameters_) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    params_changed_callback(msg);
    do_initialize_parameters_ = false;
  }

  // Update the forces and moments
  geometry_msgs::msg::WrenchStamped forces_moments = update_forces_and_torques(current_state_, current_wind_, msg.values);

  // Publish forces and moments
  forces_moments_pub_->publish(forces_moments);
}

void ForcesAndMomentsInterface::params_changed_callback(const std_msgs::msg::Bool & msg)
{
  // If true, load firmware parameters from rosflight_io
  if (msg.data) {
    // First check if rosflight_io has all the parameters. Otherwise, all the param_get service calls will be incorrect
    if (!send_check_params_service_to_firmware()) {
      return;
    }

    // Get the mixer parameters from the firmware
    get_mixer_firmware_parameters();

    // Don't invert fixedwing mixers, since the fixedwing mixers were never inverted by firmware.
    // Don't invert if primary mixer is out of range
    if (primary_mixer_ != 9 &&
        primary_mixer_ != 10 &&
        primary_mixer_ < 12 &&
        (primary_mixer_ == 11 &&
         this->get_parameter("invert_mixing_matrix").as_bool())
       ) {
      // Invert the mixer, since the mixer stored in the firmware parameters
      // is M^{-1} as defined in Small Unmanned Aircraft chapter 14, inverting it here gives M.
      invert_matrix(primary_mixing_matrix_);
      invert_matrix(secondary_mixing_matrix_);
    }

    // Get any implementation-specific parameters
    get_firmware_parameters();
  }
}

void ForcesAndMomentsInterface::get_mixer_firmware_parameters()
{
  // Get the primary and secondary mixer type
  std::string param_name = "PRIMARY_MIXER";
  double param_val = send_get_param_service_to_firmware(param_name);
  primary_mixer_ = (int) param_val;

  param_name = "SECONDARY_MIXER";
  param_val = send_get_param_service_to_firmware(param_name);
  secondary_mixer_ = (int) param_val;

  // Get the relevant mixer header parameters
  for (int i=0; i<NUM_MIXER_OUTPUTS; ++i) {
    param_name = "PRI_MIXER_OUT_" + std::to_string(i);
    param_val = send_get_param_service_to_firmware(param_name);
    mixer_header_vals_(i) = (int) param_val;
  }

  // Get the mixer matrix parameters and relevant header parameters
  for (int i=0; i<6; ++i) {
    for (int j=0; j<NUM_MIXER_OUTPUTS; ++j) {
      param_name = "PRI_MIXER_" + std::to_string(i) + "_" + std::to_string(j);
      param_val = send_get_param_service_to_firmware(param_name);
      primary_mixing_matrix_(i,j) = param_val;

      param_name = "SEC_MIXER_" + std::to_string(i) + "_" + std::to_string(j);
      param_val = send_get_param_service_to_firmware(param_name);
      secondary_mixing_matrix_(i,j) = param_val;
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Primary mixer used in sim:\n" << primary_mixing_matrix_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Secondary mixer used in sim:\n" << secondary_mixing_matrix_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Mixer headers used in sim:\n" << mixer_header_vals_);
}

double ForcesAndMomentsInterface::send_get_param_service_to_firmware(std::string param_name)
{
  // Call rosflight_io service
  // check to see if service exists
  if (!firmware_param_get_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Firmware \"param_get\" service not available! Unable to get " + param_name + " parameter! Value will be zero");
    return 0.0;
  }

  // Fill in request object
  auto req = std::make_shared<rosflight_msgs::srv::ParamGet::Request>();
  req->name = param_name;

  // Send service request and wait for response
  auto result_future = firmware_param_get_client_->async_send_request(req);
  std::future_status status = result_future.wait_for(std::chrono::milliseconds(5000));

  // Check if everything finished properly
  if (status != std::future_status::ready) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Firmware param_get service timed out! " + param_name + " value will be zero");
    return 0.0;
  }

  rosflight_msgs::srv::ParamGet::Response::SharedPtr response = result_future.get(); // Can only get it once

  if (!response->exists) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Firmware parameter " + param_name + " does not exist! Value will be zero");
    return 0.0;
  }

  return response->value;
}

bool ForcesAndMomentsInterface::send_check_params_service_to_firmware()
{
  // Call rosflight_io service
  // check to see if service exists
  if (!firmware_check_param_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Firmware \"all_params_received\" service not available!");
    return false;
  }

  // Fill in request object
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

  // Send service request and wait for response
  auto result_future = firmware_check_param_client_->async_send_request(req);
  std::future_status status = result_future.wait_for(std::chrono::milliseconds(5000));

  // Check if everything finished properly
  if (status != std::future_status::ready) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Firmware all_params_received service timed out!");
    return false;
  }

  std_srvs::srv::Trigger::Response::SharedPtr response = result_future.get(); // Can only get it once
  return response->success;
}

void ForcesAndMomentsInterface::invert_matrix(Eigen::Matrix<double, 6, NUM_MIXER_OUTPUTS> &mixer_to_invert)
{
  // Calculate the pseudoinverse of the mixing matrix using the SVD
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, NUM_MIXER_OUTPUTS>> svd(
    mixer_to_invert,
    Eigen::FullPivHouseholderQRPreconditioner | Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, NUM_MIXER_OUTPUTS, 6> Sig;
  Sig.setZero();

  // Avoid dividing by zero in the Sigma matrix
  if (svd.singularValues()[0] != 0.0) { Sig(0, 0) = 1.0 / svd.singularValues()[0]; }
  if (svd.singularValues()[1] != 0.0) { Sig(1, 1) = 1.0 / svd.singularValues()[1]; }
  if (svd.singularValues()[2] != 0.0) { Sig(2, 2) = 1.0 / svd.singularValues()[2]; }
  if (svd.singularValues()[3] != 0.0) { Sig(3, 3) = 1.0 / svd.singularValues()[3]; }
  if (svd.singularValues()[4] != 0.0) { Sig(4, 4) = 1.0 / svd.singularValues()[4]; }
  if (svd.singularValues()[5] != 0.0) { Sig(5, 5) = 1.0 / svd.singularValues()[5]; }

  // Pseudoinverse of the mixing matrix
  Eigen::Matrix<double, NUM_MIXER_OUTPUTS, 6> mixer_pinv = svd.matrixV() * Sig * svd.matrixU().transpose();

  // Save the inverted matrix
  mixer_to_invert = mixer_pinv.transpose();
}

} // namespace rosflight_sim
