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

#ifndef ROSFLIGHT_SIM_ROSFLIGHT_SIL_H
#define ROSFLIGHT_SIM_ROSFLIGHT_SIL_H

#include <stdexcept>
#include <chrono>

#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include "mavlink/mavlink.h"
#include "rosflight.h"
#include "rosflight_sim/fixedwing_forces_and_moments.hpp"
#include "rosflight_sim/mav_forces_and_moments.hpp"
#include "rosflight_sim/multirotor_forces_and_moments.hpp"
#include "rosflight_sim/sil_board.hpp"

namespace rosflight_sim
{
/**
 * @brief This class serves as the simulation loop manager. It contains a service 
 * that initializes one iteration of SIL simulation.
 */
class ROSflightSIL : public rclcpp::Node
{
public:
  ROSflightSIL();

private:
  // Timer that ticks the simulation
  rclcpp::TimerBase::SharedPtr simulation_loop_timer_;

  // Service call can be used to tick the firmware externally 
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_SIL_iteration_srvs_;

  // Service clients
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr firmware_run_client_;
  // TODO: Add the service type
  rclcpp::Client<>::SharedPtr forces_and_moments_client_;
  rclcpp::Client<>::SharedPtr dynamics_client_;

  /**
   * @brief Iterates the simulation once by calling services belonging to the 
   * SILboard, the Dynamics, and the Forces and Moments nodes
   */
  bool iterate_simulation(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                          const std_srvs::srv::Trigger::Response::SharedPtr & res);

  void take_simulation_step();

  /**
   *  @brief Declares all of the parameters with the ROS2 parameter system. Called during initialization
   */
  void declare_parameters();

  /**
   * ROS2 parameter system interface. This connects ROS2 parameters with the defined update callback,
   * parametersCallback.
   */
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  /**
   * Callback for when parameters are changed using ROS2 parameter system.
   * This takes all new changed params and updates the appropriate parameters in the params_ object.
   * @param parameters Set of updated parameters.
   * @return Service result object that tells the requester the result of the param update.
   */
  rcl_interfaces::msg::SetParametersResult
  parameters_callback(const std::vector<rclcpp::Parameter> & parameters);

  void reset_timers();

};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_ROSFLIGHT_SIL_H
