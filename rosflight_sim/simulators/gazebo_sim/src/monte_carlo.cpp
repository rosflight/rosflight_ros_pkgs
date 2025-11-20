/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2025 Ian Reid
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

#include "monte_carlo.hpp"

namespace rosflight_sim
{

MonteCarlo::MonteCarlo() : Node("monte_carlo_manager")
{
  this->declare_parameter("num_runs", 10); // Number of individual runs.
  this->declare_parameter("run_length", 30.0); // In seconds.
  
  float run_length = this->get_parameter("run_length").as_double();
  
  run_timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    std::chrono::microseconds(static_cast<long long>(run_length * 1'000'000)),
    std::bind(&MonteCarlo::start_next_run, this));

  gazebo_pause_client_ = this->create_client<std_srvs::srv::Empty>("pause_physics");
  gazebo_play_client_ = this->create_client<std_srvs::srv::Empty>("unpause_physics");
  set_estimator_state_ = this->create_client<rosflight_msgs::srv::ParamFile>("set_estimator_state");

  load_estimator_state();
  load_start_sim_state();
}

void MonteCarlo::start_next_run()
{
  // Pause the simulation. 
  pause_sim();
  // Reset the nodes.
  reset_autonomy_stack();
  // Reset the sim state.
  reset_sim_state();
  // Play the simulation. 
  play_sim();
}

void MonteCarlo::pause_sim()
{
  auto gz_pause_future = gazebo_pause_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  std::future_status status = gz_pause_future.wait_for(std::chrono::milliseconds(10));
  // TODO: Add time manager pause.
}

void MonteCarlo::play_sim()
{
  auto gazebo_play_future = gazebo_play_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  std::future_status status = gazebo_play_future.wait_for(std::chrono::milliseconds(0));
}

void MonteCarlo::reset_sim_state()
{
  auto sim_state_req = std::make_shared<rosflight_msgs::srv::SetSimState::Request>();
  
  sim_state_req->state.pose.position.set__x(start_state_.pose.position.x);
  sim_state_req->state.pose.position.set__y(start_state_.pose.position.y);
  sim_state_req->state.pose.position.set__z(start_state_.pose.position.z);

  sim_state_req->state.pose.orientation.set__w(start_state_.pose.orientation.w);
  sim_state_req->state.pose.orientation.set__x(start_state_.pose.orientation.x);
  sim_state_req->state.pose.orientation.set__y(start_state_.pose.orientation.y);
  sim_state_req->state.pose.orientation.set__z(start_state_.pose.orientation.z);
  
  sim_state_req->state.twist.angular.set__x(start_state_.twist.angular.x);
  sim_state_req->state.twist.angular.set__y(start_state_.twist.angular.y);
  sim_state_req->state.twist.angular.set__z(start_state_.twist.angular.z);

  sim_state_req->state.twist.linear.set__x(start_state_.twist.linear.x);
  sim_state_req->state.twist.linear.set__y(start_state_.twist.linear.y);
  sim_state_req->state.twist.linear.set__z(start_state_.twist.linear.z);

  auto sim_state_future = gazebo_set_sim_state_client_->async_send_request(sim_state_req);
  auto status = sim_state_future.wait_for(std::chrono::milliseconds(1000));
}

void MonteCarlo::reset_autonomy_stack()
{

  reset_estimator();
  // reset_controller();
  // reset_path_follower();
  // reset_path_manager();
  // reset_path_planner();

}

void MonteCarlo::reset_estimator()
{
  auto service_wait_for_exist = std::chrono::milliseconds(1000);
  auto service_wait_for_result = std::chrono::milliseconds(1000);

  auto req = std::make_shared<rosflight_msgs::srv::ParamFile::Request>();
  req->filename = this->get_parameter("estimator_state_filepath").as_string();
  if (!set_estimator_state_client_->wait_for_service(service_wait_for_exist)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "set_estimator_state service doesn't appear to be available.");
  }

  // Send service request and wait for response
  auto result_future = set_estimator_state_client_->async_send_request(req);
  std::future_status status = result_future.wait_for(service_wait_for_result);   // Guarantees graceful finish
  
  // Check if everything finished properly
  if (status == std::future_status::ready) {
    rosflight_msgs::srv::ParamFile::Response::SharedPtr response = result_future.get();

    if (!response->success) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Failed to set estimator state.");
    }
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "set_estimator_state timed out.");
  }
}

void MonteCarlo::load_start_sim_state()
{
  
}

void MonteCarlo::load_estimator_state()
{
  std::filesystem::path file_path(req->filename);
  std::ifstream in(file_path.string());

  in >> init_lat_;
  in >> init_lon_;
  in >> init_alt_;
  in >> init_static_;
  in >> rho_; // Set the init cond stuff.
  
  for (int i=0; i<num_states; i++) { // Set the estimated state.
    in >> xhat_(i);
  }
  
  for (int i=0; i<num_states; i++) { // Set the covariance. This goes row by row.
    for (int j=0; j<num_states; j++) {
      in >> P_(i,j);
    }
  }
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosflight_sim::MonteCarlo>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

} // end namespace rosflight_sim
