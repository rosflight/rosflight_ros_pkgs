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

#include "rosflight_msgs/msg/state.hpp"
#include "sensors.h"

namespace rosflight_sim
{

class SensorInterface : public rclcpp::Node
{
public:
  SensorInterface();

  /*
   * @brief Computes a new sensor measurement from the current state
  */
  virtual void imu_update(rosflight_msgs::msg::State state, bool motors_spinning);
  virtual void mag_update(rosflight_msgs::msg::State state);
  virtual void baro_update(rosflight_msgs::msg::State state);
  virtual void gnss_update(rosflight_msgs::msg::State state);
  virtual void sonar_update(rosflight_msgs::msg::State state);
  virtual void diff_pressure_update(rosflight_msgs::msg::State state);
  virtual void battery_update(rosflight_msgs::msg::State state);

protected:
  // Sensor noise characteristics
  std::default_random_engine bias_generator_;
  std::default_random_engine noise_generator_;
  std::normal_distribution<double> normal_distribution_;
  std::uniform_real_distribution<double> uniform_distribution_;

  // TODO: Make it so these can be configured via json
  // Noise characteristics
  double gyro_stdev_ = 0;
  double gyro_bias_walk_stdev_ = 0;
  double gyro_bias_range_ = 0;

  double acc_stdev_ = 0;
  double acc_bias_range_ = 0;
  double acc_bias_walk_stdev_ = 0;

  double baro_bias_walk_stdev_ = 0;
  double baro_stdev_ = 0;
  double baro_bias_range_ = 0;

  double mag_stdev_ = 0;
  double mag_gauss_markov_eta_[3] = {0, 0, 0};
  double k_mag_ = 0;

  double airspeed_bias_walk_stdev_ = 0;
  double airspeed_stdev_ = 0;
  double airspeed_bias_range_ = 0;

  double sonar_stdev_ = 0;
  double sonar_max_range_ = 0;
  double sonar_min_range_ = 0;

  double horizontal_gnss_stdev_ = 0;
  double vertical_gnss_stdev_ = 0;
  double gnss_velocity_stdev_ = 0;
  double gnss_gauss_markov_eta_[3] = {0, 0, 0};
  double k_gnss_ = 0;

  double mass_ = 0;
  double rho_ = 0;
  double origin_latitude_ = 0;
  double origin_longitude_ = 0;
  double origin_altitude_ = 0;

  // Sensor characteristics
  // TODO: configure these via JSON
  float imu_update_frequency_;
  float mag_update_frequency_;
  float baro_update_frequency_;
  float gnss_update_frequency_;
  float sonar_update_frequency_;
  float diff_pressure_update_frequency_;
  float battery_update_frequency_;

  // Bias
  double gyro_bias_[3] = {0, 0, 0};
  double acc_bias_[3] = {0, 0, 0};
  double mag_bias_[3] = {0, 0, 0};
  double baro_bias_ = 0;
  double airspeed_bias_ = 0;

private:
  // ROS2 interfaces
  rclcpp::Publisher<>::SharedPtr imu_pub_;
  rclcpp::Publisher<>::SharedPtr mag_pub_;
  rclcpp::Publisher<>::SharedPtr baro_pub_;
  rclcpp::Publisher<>::SharedPtr gnss_pub_;
  rclcpp::Publisher<>::SharedPtr diff_pressure_pub_;
  rclcpp::Publisher<>::SharedPtr sonar_pub_;
  rclcpp::Publisher<>::SharedPtr battery_pub_;

  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr mag_timer_;
  rclcpp::TimerBase::SharedPtr baro_timer_;
  rclcpp::TimerBase::SharedPtr gnss_timer_;
  rclcpp::TimerBase::SharedPtr diff_pressure_timer_;
  rclcpp::TimerBase::SharedPtr sonar_timer_;
  rclcpp::TimerBase::SharedPtr battery_timer_;

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

  /**
   * @brief Resets the timers if the timer frequency parameter has changed
   */
  void reset_timers();

  /**
   * @brief Timer callback to publish imu data
   */
  void imu_publish();
  void mag_publish();
  void baro_publish();
  void gnss_publish();
  void diff_pressure_publish();
  void sonar_publish();
  void battery_sim();

};

} // rosflight_sim

#endif // ROSFLIGHT_SIM_SENSOR_INTERFACE_H
