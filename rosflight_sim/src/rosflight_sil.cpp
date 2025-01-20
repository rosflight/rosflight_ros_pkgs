/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
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

#include <chrono>

#include "rosflight_sim/rosflight_sil.hpp"

using namespace std::chrono_literals;

namespace rosflight_sim
{

ROSflightSIL::ROSflightSIL()
  : rclcpp::Node("ROSflightSimManager")
{
  // Declare parameters and set up the parameter change callbacks
  declare_parameters();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ROSflightSIL::parameters_callback, this, std::placeholders::_1));

  // Initialize the timer if the parameter is set to be so
  if (this->get_parameter("use_timer").as_bool()) {
    auto sim_loop_time_us = std::chrono::microseconds(static_cast<long>(1.0 / this->get_parameter("simulation_loop_frequency").as_double() * 1e6));
    simulation_loop_timer_ = rclcpp::create_timer(this, this->get_clock(), sim_loop_time_us, std::bind(&ROSflightSIL::take_simulation_step, this));
  }

  // Initialize the service 
  run_SIL_iteration_srvs_ = this->create_service<std_srvs::srv::Trigger>(
      "rosflight_sil/iterate_simulation", std::bind(&ROSflightSIL::iterate_simulation, this,
                                                    std::placeholders::_1, std::placeholders::_2));

  // Initialize the service clients that will be used
  client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  firmware_run_client_ = this->create_client<std_srvs::srv::Trigger>("sil_board/run", rmw_qos_profile_services_default, client_cb_group_);
  forces_and_moments_client_ = this->create_client<std_srvs::srv::Trigger>("forces_and_moments/run", rmw_qos_profile_services_default, client_cb_group_);
  dynamics_client_ = this->create_client<std_srvs::srv::Trigger>("dynamics/apply_forces_and_moments", rmw_qos_profile_services_default, client_cb_group_);
}

void ROSflightSIL::declare_parameters()
{
  this->declare_parameter("simulation_loop_frequency", 400.00);
  // Determines if the sim should be run off a timer or not. Set to false if you want to call a service manually to iterate the firmware 
  // section of the simulation
  this->declare_parameter("use_timer", true);
  this->declare_parameter("service_exists_timeout_ms", 10);
  this->declare_parameter("service_result_timeout_ms", 10);
}

rcl_interfaces::msg::SetParametersResult
ROSflightSIL::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters is not a parameter of ROSflightSIL.";

  for (const auto & param : parameters) {
    if (param.get_name() == "simulation_loop_frequency") {
      reset_timers();
    }
  }

  return result;
}

void ROSflightSIL::reset_timers()
{
  if (this->get_parameter("use_timer").as_bool()) {
    simulation_loop_timer_->cancel();

    auto sim_loop_time_us = std::chrono::microseconds(static_cast<long>(1.0 / this->get_parameter("simulation_loop_frequency").as_double() * 1e6));
    simulation_loop_timer_ = rclcpp::create_timer(this, this->get_clock(), sim_loop_time_us, std::bind(&ROSflightSIL::take_simulation_step, this));
  }
}

bool ROSflightSIL::iterate_simulation(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                        const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  res->success = take_simulation_step();

  return true;
}

bool ROSflightSIL::take_simulation_step()
{
  // Run the simulation loop
  if (!call_firmware()) { return false; }
  if (!call_forces_and_moments()) { return false; }
  if (!call_propagate_dynamics()) { return false; }

  return true;
}

bool ROSflightSIL::call_firmware()
{
  auto service_wait_for_exist = std::chrono::milliseconds(this->get_parameter("service_exists_timeout_ms").as_int());
  auto service_wait_for_result = std::chrono::milliseconds(this->get_parameter("service_result_timeout_ms").as_int());

  // Check to see if service exists
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  if (!firmware_run_client_->wait_for_service(service_wait_for_exist)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "sil_board/run service not available! Aborting simulation iteration");
    return false;
  }

  // Send service request and wait for response
  auto result_future = firmware_run_client_->async_send_request(req);
  std::future_status status = result_future.wait_for(service_wait_for_result);   // Guarantees graceful finish

  // Check if everything finished properly
  if (status == std::future_status::ready) {
    if (!result_future.get()->success) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Failed to run sil_board/run service! Aborting simulation iteration");
      return false;
    }
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "sil_board/run service client timed out! Aborting simulation iteration");
    return false;
  }

  return true;
}

bool ROSflightSIL::call_forces_and_moments()
{
  auto service_wait_for_exist = std::chrono::milliseconds(this->get_parameter("service_exists_timeout_ms").as_int());
  auto service_wait_for_result = std::chrono::milliseconds(this->get_parameter("service_result_timeout_ms").as_int());

  // Check to see if service exists
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  if (!forces_and_moments_client_->wait_for_service(service_wait_for_exist)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "forces_and_moments/run service not available! Aborting simulation iteration");
    return false;
  }

  // Send service request and wait for response
  auto result_future = forces_and_moments_client_->async_send_request(req);
  std::future_status status = result_future.wait_for(service_wait_for_result);   // Guarantees graceful finish

  // Check if everything finished properly
  if (status == std::future_status::ready) {
    if (!result_future.get()->success) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Failed to run forces_and_moments/run service! Aborting simulation iteration");
      return false;
    }
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "forces_and_moments/run service client timed out! Aborting simulation iteration");
    return false;
  }

  return true;
}

bool ROSflightSIL::call_propagate_dynamics()
{
  auto service_wait_for_exist = std::chrono::milliseconds(this->get_parameter("service_exists_timeout_ms").as_int());
  auto service_wait_for_result = std::chrono::milliseconds(this->get_parameter("service_result_timeout_ms").as_int());

  // Check to see if service exists
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  if (!dynamics_client_->wait_for_service(service_wait_for_exist)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "dynamics/apply_forces_and_moments service not available! Aborting simulation iteration");
    return false;
  }

  // Send service request and wait for response
  auto result_future = dynamics_client_->async_send_request(req);
  std::future_status status = result_future.wait_for(service_wait_for_result);   // Guarantees graceful finish

  // Check if everything finished properly
  if (status == std::future_status::ready) {
    if (!result_future.get()->success) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Failed to run dynamics/apply_forces_and_moments service! Aborting simulation iteration");
      return false;
    }
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "dynamics/apply_forces_and_moments service client timed out! Aborting simulation iteration");
    return false;
  }

  return true;
}

} // namespace rosflight_sim

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosflight_sim::ROSflightSIL>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
