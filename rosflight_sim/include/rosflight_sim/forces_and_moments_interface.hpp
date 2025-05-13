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

#include <chrono>
#include <future>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rosflight_msgs/msg/pwm_output.hpp>
#include <rosflight_msgs/msg/sim_state.hpp>
#include <rosflight_msgs/srv/param_get.hpp>


namespace rosflight_sim
{
/**
 * @brief Base class for forces and moments classes for UAVs.
 */
class ForcesAndMomentsInterface : public rclcpp::Node
{
protected:
  static constexpr uint8_t NUM_TOTAL_OUTPUTS = 14;
  static constexpr uint8_t NUM_MIXER_OUTPUTS = 10;

  // Matrix to store the current mixing matrix
  Eigen::Matrix<double, 6, NUM_MIXER_OUTPUTS> primary_mixing_matrix_;
  Eigen::Matrix<double, 6, NUM_MIXER_OUTPUTS> secondary_mixing_matrix_;
  Eigen::Matrix<int, 1, NUM_MIXER_OUTPUTS> mixer_header_vals_;
  int primary_mixer_ = 255;
  int secondary_mixer_ = 255;

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
    // Double check the user entered it in correctly
    if (max < min) {
      double tmp = min;
      min = max;
      max = tmp;
    }

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

private:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr forces_moments_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::SimState>::SharedPtr truth_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::PwmOutput>::SharedPtr firware_out_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr params_changed_sub_;

  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::Client<rosflight_msgs::srv::ParamGet>::SharedPtr firmware_param_get_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr firmware_check_param_client_;

  // Persistent variables
  rosflight_msgs::msg::SimState current_state_;
  geometry_msgs::msg::Vector3Stamped current_wind_;
  bool do_initialize_parameters_ = false;

  // Callbacks
  void state_callback(const rosflight_msgs::msg::SimState & msg);
  void wind_callback(const geometry_msgs::msg::Vector3Stamped & msg);
  void firmware_output_callback(const rosflight_msgs::msg::PwmOutput & msg);
  void params_changed_callback(const std_msgs::msg::Bool & msg);

  void get_mixer_firmware_parameters();
  double send_get_param_service_to_firmware(std::string param_name);
  bool send_check_params_service_to_firmware();
  void invert_matrix(Eigen::Matrix<double, 6, NUM_MIXER_OUTPUTS> &mixer_to_invert);

public:

  ForcesAndMomentsInterface();

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
                                                                      std::array<uint16_t, NUM_TOTAL_OUTPUTS> act_cmds) = 0;

  /**
  * @brief Interface function for pulling the current firmware parameters from rosflight_io.
  * Mixer is pulled by default before this function gets called.
  */
  virtual void get_firmware_parameters() {};
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_MAV_FORCES_AND_MOMENTS_H
