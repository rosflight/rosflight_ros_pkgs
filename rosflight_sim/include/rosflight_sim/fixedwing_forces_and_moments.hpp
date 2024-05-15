/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland and Ian Reid, AeroVironment Inc.
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

#ifndef ROSFLIGHT_SIM_FIXEDWING_FORCES_AND_MOMENTS_H
#define ROSFLIGHT_SIM_FIXEDWING_FORCES_AND_MOMENTS_H

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rosflight_sim/mav_forces_and_moments.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace rosflight_sim
{
/**
 * @brief This class contains the forces and moments calculations used for fixedwing simulations in
 * Gazebo. It uses the dynamic model described in Small Unmanned Aircraft: Theory and Practice by
 * Dr. Randy Beard and Dr. Tim McLain. Aerodynamic parameters can be configured at runtime in the
 * fixedwing_dynamics.yaml parameter file.
 *
 * @note Default values for parameters are not provided as parameters are interdependent on each
 * other and need to be provided as a set. Notifying the user of missing parameters helps avoid
 * inadvertently using an incomplete set of parameters.
 */
class Fixedwing : public MAVForcesAndMoments
{
private:
  rclcpp::Node::SharedPtr node_;

  // physical parameters
  double rho_;

  // aerodynamic coefficients
  struct WingCoeff
  {
    double S;
    double b;
    double c;
    double M;
    double epsilon;
    double alpha0;
  } wing_;

  // Propeller Coefficients 
  struct PropCoeff
  {
    double D_prop;
    double CT_0;
    double CT_1;
    double CT_2;
    double CQ_0;
    double CQ_1;
    double CQ_2;
  } prop_;
  
  // Motor Coefficients 
  struct MotorCoeff
  {
    double KV;
    double KQ;
    double V_max;
    double R_motor;
    double I_0;
  } motor_;

  // Lift Coefficients
  struct LiftCoeff
  {
    double O;
    double alpha;
    double beta;
    double p;
    double q;
    double r;
    double delta_a;
    double delta_e;
    double delta_r;
  };

  LiftCoeff CL_;
  LiftCoeff CD_;
  LiftCoeff Cm_;
  LiftCoeff CY_;
  LiftCoeff Cell_;
  LiftCoeff Cn_;

  double servo_tau_;

  // not constants
  // actuators
  struct Actuators
  {
    double e;
    double a;
    double r;
    double t;
  } delta_;

  Actuators delta_prev_;
  Actuators delta_prev_command_;

  // wind
  Eigen::Vector3d wind_;

  /**
   * @brief Declares ROS parameters. Must be called in the constructor.
   */
  void declare_fixedwing_params();

  /**
  * @brief Updates class's aerodynamic parameters with parameters from ROS. Will print a ROS error message
  * for any missing parameters.
  */
  void update_params_from_ROS();
  
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr forces_moments_pub_;

public:
  /**
   * @param node ROS2 node to obtain parameters from. Usually the node provided by the Gazebo model
   * plugin.
   */
  explicit Fixedwing(rclcpp::Node::SharedPtr node);
  ~Fixedwing();

  /**
   * @brief Calculates forces and moments based on current state and aerodynamic forces.
   *
   * @param x Current state of aircraft
   * @param act_cmds Actuator commands
   * @return 6x1 eigen matrix of calculated forces and moments
   */
  Eigen::Matrix<double, 6, 1> update_forces_and_torques(CurrentState x,
                                                        const int act_cmds[]) override;
  /**
   * @brief Sets the wind speed to use when calculating the forces and moments.
   *
   * @param wind Eigen vector of wind speeds
   */
  void set_wind(Eigen::Vector3d wind) override;
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_FIXEDWING_FORCES_AND_MOMENTS_H
