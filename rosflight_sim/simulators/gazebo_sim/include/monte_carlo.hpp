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


#ifndef ROSFLIGHT_SIM_MONTE_CARLO_GAZEBO_H
#define ROSFLIGHT_SIM_MONTE_CARLO_GAZEBO_H

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rosflight_msgs/srv/set_sim_state.hpp>
#include <rosflight_msgs/srv/param_file.hpp>

#include <filesystem>
#include <fstream>

namespace rosflight_sim
{

class MonteCarlo : public rclcpp::Node
{
public:
  MonteCarlo();

private:
  rclcpp::TimerBase::SharedPtr run_timer_;

  void load_estimator_state(); // TODO: Make this load from the filename the parameters.
  void load_start_sim_state(); // TODO: Make this load the sim state from some truth source.
  void reset_estimator();

  void start_next_run();
  void pause_sim();
  void play_sim();
  void reset_sim_state();
  void reset_autonomy_stack();
  
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_pause_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_play_client_;
  rclcpp::Client<rosflight_msgs::srv::ParamFile>::SharedPtr set_estimator_state_client_;
  
  rclcpp::Client<rosflight_msgs::srv::SetSimState>::SharedPtr gazebo_set_sim_state_client_;

  rosflight_msgs::msg::SimState start_state_;
};

} // rosflight_sim

#endif // ROSFLIGHT_SIM_MONTE_CARLO_GAZEBO_H
