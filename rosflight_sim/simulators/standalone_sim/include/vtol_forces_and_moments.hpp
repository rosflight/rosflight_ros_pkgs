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

#include <cmath>
#include <cstdint>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <Eigen/Geometry>

#include "rosflight_sim/forces_and_moments_interface.hpp"
#include "rosflight_msgs/msg/sim_state.hpp"

namespace rosflight_sim
{
/**
 * @brief This class contains the forces and moments calculations used for VTOL simulations.
 *
 * @note Default values for parameters are not provided as parameters are interdependent on each
 * other and need to be provided as a set. Notifying the user of missing parameters helps avoid
 * inadvertently using an incomplete set of parameters.
 */
class VTOL : public ForcesAndMomentsInterface
{
private:
  bool paramsHaveChanged_;
  // Shared parameters
  double V_max_;
  double rho_;

  //Multi-rotor variables--------------------------

  struct MR_Prop
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

  struct MR_Motor
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
    MR_Prop prop;
  };

  int num_rotors_;
  std::vector<MR_Motor> motors_;
  double cross_sectional_area_; // m^2
  double mr_CD_;
  double CD_induced_;
  std::vector<double> ground_effect_coeffs_;

  //Fixed-wing variables--------------------------------

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
  struct FW_Prop
  {
    double D_prop;
    double CT_0;
    double CT_1;
    double CT_2;
    double CQ_0;
    double CQ_1;
    double CQ_2;
  } fw_prop_;

  // Motor Coefficients
  struct FW_Motor
  {
    double KV;
    double KQ;
    double V_max;
    double R_motor;
    double I_0;
  } fw_motor_;

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

  double max_deflection_angle_; // What is this used for?
  double servo_tau_;
  double servo_refresh_rate_; // refresh rate TODO: find a way to programmatically set this.
  double max_aileron_deflection_angle_;
  double max_elevator_deflection_angle_;
  double max_rudder_deflection_angle_;

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

  /**
   * @brief Declares ROS parameters. Must be called in the constructor.
   */
  void declare_vtol_params();

  void update_params_from_ROS();

  // Parameters callback that flags params have changed
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> & parameters);


public:
  explicit VTOL();
  ~VTOL();

  /**
   * @brief Calculates forces and moments based on current state and aerodynamic forces.
   *
   * @param x Current state of aircraft
   * @param wind 3-vector of current wind acting on the aircraft in the inertial frame
   * @param act_cmds Array of actuator commands
   * @return geometry_msgs::msg::WrenchStamped object with calculated forces and moments
   */
  geometry_msgs::msg::WrenchStamped update_forces_and_torques(rosflight_msgs::msg::SimState x,
                                                              geometry_msgs::msg::Vector3Stamped wind,
                                                              std::array<uint16_t, NUM_TOTAL_OUTPUTS> act_cmds) override;

  /**
  * @brief Queries rosflight_io for any changed parameters
  */
  void get_firmware_parameters() override;
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_MULTIROTOR_FORCES_AND_MOMENTS_H
