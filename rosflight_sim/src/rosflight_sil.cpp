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

#include "rosflight_sim/rosflight_sil.hpp"

namespace rosflight_sim
{

ROSflightSIL::ROSflightSIL()
  : rclcpp::Node("ROSflightSimManager")
{
  // Initialize the service 
  run_SIL_iteration_srvs_ = this->create_service<>(
      "rosflight_sil/iterate_simulation", std::bind(&ROSflightSIL::iterate_simulation, this,
                                                    std::placeholders::_1, std::placeholders::_2));

  // Initialize the service clients that will be used
  client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  firmware_run_client_ = this->create_client<std_srvs::srv::Trigger>("sil_board/run", rmw_qos_profile_services_default, client_cb_group_);
  forces_and_moments_client_ = this->create_client<>("forces_and_moments/run", rmw_qos_profile_services_default, client_cb_group_);
  dynamics_client_ = this->create_client<>("dynamics/apply_forces_and_moments", rmw_qos_profile_services_default, client_cb_group_);
}

bool iterate_simulation(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                        const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  auto service_wait_for_exist = 10ms;   // TODO: too short?
  auto service_wait_for_result = 10ms;  // TODO: too short?

  // Run a simulation loop

  // Run the firmware 

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  // Check to see if service exists
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

  // Compute forces and moments

  // TODO: Add the service type
  auto req = std::make_shared<::Request>();
  // Check to see if service exists
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

  // Propagate dynamics

  // TODO: Add the service type
  auto req = std::make_shared<::Request>();
  // Check to see if service exists
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
