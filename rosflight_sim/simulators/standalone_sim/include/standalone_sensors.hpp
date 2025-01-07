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


#ifndef ROSFLIGHT_SIM_STANDALONE_SENSORS_H
#define ROSFLIGHT_SIM_STANDALONE_SENSORS_H

#include "rosflight_sim/sensor_interface.hpp"

namespace rosflight_sim
{

class StandaloneSensors : public SensorInterface
{
public:
  StandaloneSensors();

  /*
   * @brief Computes a new sensor measurement from the current state
  */
  void imu_update(rosflight_msgs::msg::State state, geometry_msgs::msg::TwistStamped forces) override;
  void imu_temperature_update(rosflight_msgs::msg::State state) override;
  void mag_update(rosflight_msgs::msg::State state) override;
  void baro_update(rosflight_msgs::msg::State state) override;
  void gnss_update(rosflight_msgs::msg::State state) override;
  void sonar_update(rosflight_msgs::msg::State state) override;
  void diff_pressure_update(rosflight_msgs::msg::State state) override;
  void battery_update(rosflight_msgs::msg::State state) override;

private:
  // Sensor noise
  std::default_random_engine bias_generator_;
  std::default_random_engine noise_generator_;
  std::normal_distribution<double> normal_distribution_;
  std::uniform_real_distribution<double> uniform_distribution_;

  Eigen::Vector3d mag_gauss_markov_eta_[3] = Eigen::Vector3d::Zeros();
  Eigen::Vector3d inertial_magnetic_field_;

  double rho_ = 1.225;

  // Bias
  double gyro_bias_[3] = {0, 0, 0};
  double acc_bias_[3] = {0, 0, 0};
  double mag_bias_[3] = {0, 0, 0};
  double baro_bias_ = 0;
  double airspeed_bias_ = 0;

  /**
   *  @brief Declares all of the parameters with the ROS2 parameter system. Called during initialization
   */
  void declare_parameters();

  /**
   * @brief Initializes the noise generators and the biases for the sensors
   */
  void initialize_sensors();

};

} // rosflight_sim

#endif // ROSFLIGHT_SIM_STANDALONE_SENSORS_H
