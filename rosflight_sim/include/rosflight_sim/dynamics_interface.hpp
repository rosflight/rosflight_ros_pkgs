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


#ifndef ROSFLIGHT_SIM_SENSOR_INTERFACE_H
#define ROSFLIGHT_SIM_SENSOR_INTERFACE_H

#include <rclcpp/rclcpp.hpp>

#include "sensors.h"

namespace rosflight_sim
{

class DynamicsInterface : public rclcpp::Node
{
public:
  DynamicsInterface();

private:
  virtual void apply_forces_and_torques() = 0;

  // ROS2 interfaces
  rclcpp::Publisher<>::SharedPtr truth_state_pub_;

  rclcpp::Service<>::SharedPtr dynamics_update_srvs_;

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
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

};

#endif // ROSFLIGHT_SIM_SENSOR_INTERFACE_H
