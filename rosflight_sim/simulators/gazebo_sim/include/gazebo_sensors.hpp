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


#ifndef ROSFLIGHT_SIM_GAZEBO_SENSORS_H
#define ROSFLIGHT_SIM_GAZEBO_SENSORS_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include "rosflight_sim/sensor_interface.hpp"
#include "gazebo_sim/include/gz_compat.hpp"

namespace rosflight_sim
{

class GazeboSensors : public SensorInterface
{
private:
  // Gazebo objects
  GazeboVector gravity_;
  // TODO: initialize these
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;

  GazeboVector inertial_magnetic_field_;

public:
  GazeboSensors();

  /*
   *  @brief Initializes the sensors within Gazebo using the provided sensor parameters
  */
  void sensors_init() override;

  /*
   * @brief Returns the sensor measurement simulated from the current state
  */
  bool imu_read(float accel[3], float * temperature, float gyro[3], uint64_t * time_us) override;
  bool mag_read(float mag[3]) override;
  bool baro_read(float * pressure, float * temperature) override;
  bool gnss_read(rosflight_firmware::GNSSData * gnss,
                         rosflight_firmware::GNSSFull * gnss_full) override;
  bool sonar_read(float * range) override;
  bool diff_pressure_read(float * diff_pressure, float * temperature) override;
  bool battery_read(float * voltage, float * current) override;

  /*
   * @brief Computes a new sensor measurement from the current state
  */
  void imu_update(rosflight_msgs::msg::State state, bool motors_spinning) override;
  void mag_update(rosflight_msgs::msg::State state) override;
  void baro_update(rosflight_msgs::msg::State state) override;
  void gnss_update(rosflight_msgs::msg::State state) override;
  void sonar_update(rosflight_msgs::msg::State state) override;
  void diff_pressure_update(rosflight_msgs::msg::State state) override;
  void battery_update(rosflight_msgs::msg::State state) override;

  /**
   * @brief Initializes the firmware with ROS parameters and Gazebo data types.
   *
   * @param link Gazebo link pointer, provided by ModelPlugin::Load function
   * @param world Gazebo world pointer, provided by ModelPlugin::Load function
   * @param model Gazebo model pointer, provided by ModelPlugin::Load function
   * @param node ROS node pointer, provided by ModelPlugin::Load function
   * @param mav_type Simulation type
   */
  void gazebo_setup(gazebo::physics::LinkPtr link, gazebo::physics::WorldPtr world,
                    gazebo::physics::ModelPtr model, rclcpp::Node::SharedPtr node,
                    std::string mav_type);
  inline const int * get_outputs() const { return pwm_outputs_; }
  gazebo::common::SphericalCoordinates sph_coord_;
};

} // rosflight_sim

#endif // ROSFLIGHT_SIM_GAZEBO_SENSORS_H
