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

#ifndef ROSFLIGHT_SIM_MAV_FORCES_AND_MOMENTS_H
#define ROSFLIGHT_SIM_MAV_FORCES_AND_MOMENTS_H

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rosflight_msgs/msg/pwm_output.hpp>
#include <rosflight_msgs/msg/sim_state.hpp>


namespace rosflight_sim
{
/**
 * @brief Base class for forces and moments classes for UAVs.
 */
class MAVForcesAndMoments : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr forces_moments_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::SimState>::SharedPtr truth_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::PwmOutput>::SharedPtr firware_out_sub_;

  // Persistent variables
  rosflight_msgs::msg::SimState current_state_;
  geometry_msgs::msg::Vector3Stamped current_wind_;

  // Callbacks
  void state_callback(const rosflight_msgs::msg::SimState & msg);
  void wind_callback(const geometry_msgs::msg::Vector3Stamped & msg);
  void firmware_output_callback(const rosflight_msgs::msg::PwmOutput & msg);

protected:
  /**
   * @brief Saturation function for actuator commands.
   *
   * @param x Unsaturated command
   * @param max Max allowable command value
   * @param min Min allowable command value
   * @return Saturated command
   */
  static double sat(double x, double max, double min)
  {
    if (x > max) {
      return max;
    } else if (x < min) {
      return min;
    } else {
      return x;
    }
  }

  /**
   * @brief Determine the largest value between two possible values.
   *
   * @param x Value to compare
   * @param y Value to compare
   * @return x or y, whichever is largest
   */
  static double max(double x, double y) { return (x > y) ? x : y; }

public:

  MAVForcesAndMoments();

  /**
   * @brief Interface function for calculating the current MAV forces and moments.
   *
   * @param x Current state of MAV
   * @param wind Current wind acting on the MAV
   * @param act_cmds Current MAV commands
   * @return Calculated forces and moments
   */
  virtual geometry_msgs::msg::WrenchStamped update_forces_and_torques(rosflight_msgs::msg::SimState x,
                                                                      geometry_msgs::msg::Vector3Stamped wind,
                                                                      std::array<uint16_t, 14> act_cmds) = 0;
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_MAV_FORCES_AND_MOMENTS_H
