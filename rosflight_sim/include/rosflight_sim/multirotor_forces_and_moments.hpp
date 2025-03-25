/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
 * Copyright (c) 2024 Ian Reid, BYU MAGICC Lab.
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

#ifndef ROSFLIGHT_SIM_MULTIROTOR_FORCES_AND_MOMENTS_H
#define ROSFLIGHT_SIM_MULTIROTOR_FORCES_AND_MOMENTS_H

#include <cstdint>

#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rosflight_sim/mav_forces_and_moments.hpp"
#include "rosflight_msgs/msg/sim_state.hpp"

namespace rosflight_sim
{
/**
 * @brief This class contains the forces and moments calculations used for multirotor simulations.
 *
 * @note Default values for parameters are not provided as parameters are interdependent on each
 * other and need to be provided as a set. Notifying the user of missing parameters helps avoid
 * inadvertently using an incomplete set of parameters.
 */
class Multirotor : public MAVForcesAndMoments
{
private:
  struct Prop
  {
    // Prop thrust constant coeffs.
    double CT_0;
    double CT_1;
    double CT_2;
    
    // Prop torque constant coeffs.
    double CQ_0;
    double CQ_1;
    double CQ_2;

    double diam;
  };

  struct Motor
  {
    double dist;
    // The motor's radial angle from the body north unit vector in radians,
    // for example: quad copter x frame this is 45,135,225,315, except in radians. 
    double radial_angle;
    // The direction that the motor spins, (-1 is clockwise and 1 is counter clocwise, looking from above vehicle).
    int64_t direction;
    // The current command for the motor, from 0 to 1
    double command;
    // Resistance of motor.
    double R;
    // Torque constant of motor.
    double KQ;
    // No load current draw of motor.
    double I_0;
    // Prop data
    Prop prop;
  };

  int num_rotors_;
  std::vector<Motor> motors_;

  /**
   * @brief Declares ROS parameters. Must be called in the constructor.
   */
  void declare_multirotor_params();

public:
  /**
   * @param node ROS2 node to obtain parameters from. Usually the node provided by the Gazebo model
   * plugin.
   */
  explicit Multirotor();
  ~Multirotor();

  /**
   * @brief Calculates forces and moments based on current state and aerodynamic forces.
   *
   * @param x Current state of aircraft
   * @param act_cmds Actuator commands
   * @return 6x1 eigen matrix of calculated forces and moments
   */
  geometry_msgs::msg::WrenchStamped update_forces_and_torques(rosflight_msgs::msg::SimState x,
                                                              geometry_msgs::msg::Vector3Stamped wind,
                                                              std::array<uint16_t, 14> act_cmds) override;
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_MULTIROTOR_FORCES_AND_MOMENTS_H
