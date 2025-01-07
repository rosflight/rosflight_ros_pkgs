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

#include <eigen3/Eigen/Core>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rosflight_msgs/msg/state.hpp"
#include "sensors.h"

namespace rosflight_sim
{

class SensorInterface : public rclcpp::Node
{
public:
  SensorInterface();

protected:
  // Sensor characteristics
  float imu_update_frequency_;
  float mag_update_frequency_;
  float baro_update_frequency_;
  float gnss_update_frequency_;
  float sonar_update_frequency_;
  float diff_pressure_update_frequency_;
  float battery_update_frequency_;

private:
  /*
   * @brief Computes a new sensor measurement from the current state
  */
  virtual sensor_msgs::msg::Imu imu_update(const rosflight_msgs::msg::State & state, const geometry_msgs::msg::TwistStamped & forces);
  virtual sensor_msgs::msg::Temperature imu_temperature_update(const rosflight_msgs::msg::State & state);
  virtual sensor_msgs::msg::MagneticField mag_update(const rosflight_msgs::msg::State & state);
  virtual rosflight_msgs::msg::Barometer baro_update(const rosflight_msgs::msg::State & state);
  virtual rosflight_msgs::msg::GNSS gnss_update(const rosflight_msgs::msg::State & state);
  virtual rosflight_msgs::msg::GNSSFull gnss_full_update(const rosflight_msgs::msg::State & state);
  virtual sensor_msgs::msg::Range sonar_update(const rosflight_msgs::msg::State & state);
  virtual rosflight_msgs::msg::Airspeed diff_pressure_update(const rosflight_msgs::msg::State & state);
  virtual rosflight_msgs::msg::BatteryStatus battery_update(const rosflight_msgs::msg::State & state);

  // ROS2 interfaces
  rclcpp::Publisher<sensor_msgs:msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temperature_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::Barometer>::SharedPtr baro_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::GNSS>::SharedPtr gnss_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::GNSSFull>::SharedPtr gnss_full_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::Airspeed>::SharedPtr diff_pressure_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::BatteryStatus>::SharedPtr battery_pub_;

  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr mag_timer_;
  rclcpp::TimerBase::SharedPtr baro_timer_;
  rclcpp::TimerBase::SharedPtr gnss_timer_;
  rclcpp::TimerBase::SharedPtr diff_pressure_timer_;
  rclcpp::TimerBase::SharedPtr sonar_timer_;
  rclcpp::TimerBase::SharedPtr battery_timer_;

  rclcpp::Subscription<rosflight_msgs::msg::SimState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr forces_sub_;
  rosflight_msgs::msg::SimState current_state_;
  geometry_msgs::msg::TwistStamped current_forces_;

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

  /**
   * @brief Subscription to the truth state. Used to create sensor information.
   */
  void sim_state_callback(const rosflight_msgs::msg::SimState & msg);

  /**
   * @brief Subscription to the forces and moments. Used to create sensor information.
   */
  void forces_moments_callback(const geometry_msgs::msg::TwistStamped & msg);
};

} // rosflight_sim

#endif // ROSFLIGHT_SIM_SENSOR_INTERFACE_H
