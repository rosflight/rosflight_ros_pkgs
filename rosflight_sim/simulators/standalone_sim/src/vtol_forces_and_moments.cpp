/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
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


/* TODO
- Make sure all declared variables are updates in update() and called locally
- Combine forces and moments from mr and fw into one return (make fw_forces and fw_torques)
*/

#include "vtol_forces_and_moments.hpp"

namespace rosflight_sim
{
VTOL::VTOL()
    : ForcesAndMomentsInterface()
    , paramsHaveChanged_(false)
    //multirotor variables
    , num_rotors_(0)
    //fixedwing variables
    , rho_(0)
    , wing_()
    , fw_prop_()
    , fw_motor_()
    , CL_()
    , CD_()
    , Cm_()
    , CY_()
    , Cell_()
    , Cn_()
    , delta_()
{
  // Declare parameters
  declare_vtol_params();

  // Load all parameters from ROS
  update_params_from_ROS();
}

VTOL::~VTOL() = default;

void VTOL::declare_vtol_params()
{
  // TODO: Should we try to sync these with the values in the firmware?
  //  - The firmware won't always have these values
  //  - The firmware may not use these values even if it has them (based on USE_MOTOR_PARAMS)
  //  - The firmware uses a constant CT, CQ, not the quadratic coeffs
  //  - The firmware doesn't have values like CD_induced or ground_effect
  
  // Multirotor params--------------------------------------
  this->declare_parameter("num_rotors", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("mr_CT_0", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("mr_CT_1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("mr_CT_2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("mr_CQ_0", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("mr_CQ_1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("mr_CQ_2", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("mr_KQ", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("V_max", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("mr_R_motor", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("mr_I_0", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("rotor_dists", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("rotor_radial_angles", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("rotor_rotation_directions", rclcpp::PARAMETER_INTEGER_ARRAY);

  this->declare_parameter("mr_D_prop", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("rho", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("mr_CD", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("A_c", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("CD_induced", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("ground_effect", rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Fixedwing params------------------------------------------------
  this->declare_parameter("mass", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("wing_s", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wing_b", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wing_c", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wing_M", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wing_epsilon", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wing_alpha0", rclcpp::PARAMETER_DOUBLE);

  // RC command channel configuration
  this->declare_parameter("max_aileron_deflection_angle", 40.);
  this->declare_parameter("max_elevator_deflection_angle", 40.);
  this->declare_parameter("max_rudder_deflection_angle", 40.);

  this->declare_parameter("C_L_O", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_L_alpha", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_L_beta", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_L_p", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_L_q", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_L_r", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_L_delta_a", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_L_delta_e", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_L_delta_r", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("C_D_O", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_D_alpha", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_D_beta", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_D_p", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_D_q", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_D_r", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_D_delta_a", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_D_delta_e", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_D_delta_r", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("C_ell_O", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_ell_alpha", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_ell_beta", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_ell_p", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_ell_q", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_ell_r", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_ell_delta_a", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_ell_delta_e", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_ell_delta_r", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("C_m_O", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_m_alpha", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_m_beta", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_m_p", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_m_q", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_m_r", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_m_delta_a", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_m_delta_e", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_m_delta_r", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("C_n_O", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_n_alpha", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_n_beta", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_n_p", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_n_q", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_n_r", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_n_delta_a", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_n_delta_e", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_n_delta_r", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("C_Y_O", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_Y_alpha", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_Y_beta", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_Y_p", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_Y_q", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_Y_r", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_Y_delta_a", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_Y_delta_e", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("C_Y_delta_r", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("fw_D_prop", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("fw_CT_0", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("fw_CT_1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("fw_CT_2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("fw_CQ_0", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("fw_CQ_1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("fw_CQ_2", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("KV", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("fw_KQ", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("fw_R_motor", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("fw_I_0", rclcpp::PARAMETER_DOUBLE);

  // Servo time delay parameters
  this->declare_parameter("servo_refresh_rate", 0.003);
  this->declare_parameter("servo_tau", 0.01);
}

void VTOL::update_params_from_ROS()
{
  // Multirotor params -----------------------------------------------
  if (!this->get_parameter("V_max", V_max_)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'V_max' not defined");
  }
  
  if (!this->get_parameter("num_rotors", num_rotors_)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'num_rotors' not defined");
  }

  /* Load Rotor Configuration */
  motors_.resize(num_rotors_);

  MR_Prop prop_;

  if (!this->get_parameter("mr_CT_0", prop_.CT_0)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CT_0' not defined");
  }
  if (!this->get_parameter("mr_CT_1", prop_.CT_1)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CT_1' not defined");
  }
  if (!this->get_parameter("mr_CT_2", prop_.CT_2)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CT_2' not defined");
  }

  if (!this->get_parameter("mr_CQ_0", prop_.CQ_0)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CQ_0' not defined");
  }
  if (!this->get_parameter("mr_CQ_1", prop_.CQ_1)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CQ_1' not defined");
  }
  if (!this->get_parameter("mr_CQ_2", prop_.CQ_2)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CQ_2' not defined");
  }

  if (!this->get_parameter("mr_D_prop", prop_.diam)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'D_prop' not defined");
  }

  std::vector<double> dists;
  if (!this->get_parameter("rotor_dists", dists)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'rotor_dists' not defined");
  }
  std::vector<double> angles;
  if (!this->get_parameter("rotor_radial_angles", angles)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'rotor_radial_angles' not defined");
  }
  std::vector<int64_t> rotation_dirs;
  if (!this->get_parameter("rotor_rotation_directions", rotation_dirs)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'rotor_rotation_directions' not defined");
  }

  MR_Motor motor_characteristics_;
  motor_characteristics_.prop = prop_;
  if (!this->get_parameter("mr_R_motor", motor_characteristics_.R)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'R_motor' not defined");
  }
  if (!this->get_parameter("mr_I_0", motor_characteristics_.I_0)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'I_0' not defined");
  }
  if (!this->get_parameter("mr_KQ", motor_characteristics_.KQ)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'KQ' not defined");
  }


  for (int i = 0; i < num_rotors_; i++) {
    MR_Motor motor = motor_characteristics_;
    motor.dist = dists[i];
    motor.radial_angle = angles[i] * M_PI / 180.0;
    motor.direction = rotation_dirs[i];
    motor.command = 0.0;
    motors_[i] = motor;
  }

  // Fixedwing params ---------------------------------------
  if (!this->get_parameter("rho", rho_)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'rho' not defined");
  }

  // Wing Geometry
  if (!this->get_parameter("wing_s", wing_.S)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'wing_s' not defined");
  }
  if (!this->get_parameter("wing_b", wing_.b)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'wing_b' not defined");
  }
  if (!this->get_parameter("wing_c", wing_.c)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'wing_c' not defined");
  }
  if (!this->get_parameter("wing_M", wing_.M)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'wing_M' not defined");
  }
  if (!this->get_parameter("wing_epsilon", wing_.epsilon)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'wing_epsilon' not defined");
  }
  if (!this->get_parameter("wing_alpha0", wing_.alpha0)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'wing_alpha0' not defined");
  }

  // Propeller Coefficients
  if (!this->get_parameter("fw_D_prop", fw_prop_.D_prop)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'D_prop' not defined");
  }
  if (!this->get_parameter("fw_CT_0", fw_prop_.CT_0)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CT_0' not defined");
  }
  if (!this->get_parameter("fw_CT_1", fw_prop_.CT_1)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CT_1' not defined");
  }
  if (!this->get_parameter("fw_CT_2", fw_prop_.CT_2)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CT_2' not defined");
  }
  if (!this->get_parameter("fw_CQ_0", fw_prop_.CQ_0)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CQ_0' not defined");
  }
  if (!this->get_parameter("fw_CQ_1", fw_prop_.CQ_1)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CQ_1' not defined");
  }
  if (!this->get_parameter("fw_CQ_2", fw_prop_.CQ_2)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CQ_2' not defined");
  }

  // Motor Coefficients
  if (!this->get_parameter("KV", fw_motor_.KV)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'KV' not defined");
  }
  if (!this->get_parameter("fw_KQ", fw_motor_.KQ)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'KQ' not defined");
  }
  if (!this->get_parameter("V_max", fw_motor_.V_max)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'V_max' not defined");
  }
  if (!this->get_parameter("fw_R_motor", fw_motor_.R_motor)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'R_motor' not defined");
  }
  if (!this->get_parameter("fw_I_0", fw_motor_.I_0)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'I_0' not defined");
  }

  if (!this->get_parameter("servo_tau", servo_tau_)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'servo_tau' not defined");
  }

  // Lift Params
  if (!this->get_parameter("C_L_O", CL_.O)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_L_O' not defined");
  }
  if (!this->get_parameter("C_L_alpha", CL_.alpha)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_L_alpha' not defined");
  }
  if (!this->get_parameter("C_L_beta", CL_.beta)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_L_beta' not defined");
  }
  if (!this->get_parameter("C_L_p", CL_.p)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_L_p' not defined");
  }
  if (!this->get_parameter("C_L_q", CL_.q)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_L_q' not defined");
  }
  if (!this->get_parameter("C_L_r", CL_.r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_L_r' not defined");
  }
  if (!this->get_parameter("C_L_delta_a", CL_.delta_a)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_L_delta_a' not defined");
  }
  if (!this->get_parameter("C_L_delta_e", CL_.delta_e)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_L_delta_e' not defined");
  }
  if (!this->get_parameter("C_L_delta_r", CL_.delta_r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_L_delta_r' not defined");
  }

  // Drag Params
  if (!this->get_parameter("C_D_O", CD_.O)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_D_O' not defined");
  }
  if (!this->get_parameter("C_D_alpha", CD_.alpha)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_D_alpha' not defined");
  }
  if (!this->get_parameter("C_D_beta", CD_.beta)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_D_beta' not defined");
  }
  if (!this->get_parameter("C_D_p", CD_.p)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_D_p' not defined");
  }
  if (!this->get_parameter("C_D_q", CD_.q)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_D_q' not defined");
  }
  if (!this->get_parameter("C_D_r", CD_.r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_D_r' not defined");
  }
  if (!this->get_parameter("C_D_delta_a", CD_.delta_a)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_D_delta_a' not defined");
  }
  if (!this->get_parameter("C_D_delta_e", CD_.delta_e)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_D_delta_e' not defined");
  }
  if (!this->get_parameter("C_D_delta_r", CD_.delta_r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_D_delta_r' not defined");
  }

  // ell Params (x axis moment)
  if (!this->get_parameter("C_ell_O", Cell_.O)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_ell_O' not defined");
  }
  if (!this->get_parameter("C_ell_alpha", Cell_.alpha)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_ell_alpha' not defined");
  }
  if (!this->get_parameter("C_ell_beta", Cell_.beta)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_ell_beta' not defined");
  }
  if (!this->get_parameter("C_ell_p", Cell_.p)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_ell_p' not defined");
  }
  if (!this->get_parameter("C_ell_q", Cell_.q)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_ell_q' not defined");
  }
  if (!this->get_parameter("C_ell_r", Cell_.r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_ell_r' not defined");
  }
  if (!this->get_parameter("C_ell_delta_a", Cell_.delta_a)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_ell_delta_a' not defined");
  }
  if (!this->get_parameter("C_ell_delta_e", Cell_.delta_e)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_ell_delta_e' not defined");
  }
  if (!this->get_parameter("C_ell_delta_r", Cell_.delta_r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_ell_delta_r' not defined");
  }

  // m Params (y axis moment)
  if (!this->get_parameter("C_m_O", Cm_.O)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_m_O' not defined");
  }
  if (!this->get_parameter("C_m_alpha", Cm_.alpha)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_m_alpha' not defined");
  }
  if (!this->get_parameter("C_m_beta", Cm_.beta)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_m_beta' not defined");
  }
  if (!this->get_parameter("C_m_p", Cm_.p)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_m_p' not defined");
  }
  if (!this->get_parameter("C_m_q", Cm_.q)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_m_q' not defined");
  }
  if (!this->get_parameter("C_m_r", Cm_.r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_m_r' not defined");
  }
  if (!this->get_parameter("C_m_delta_a", Cm_.delta_a)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_m_delta_a' not defined");
  }
  if (!this->get_parameter("C_m_delta_e", Cm_.delta_e)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_m_delta_e' not defined");
  }
  if (!this->get_parameter("C_m_delta_r", Cm_.delta_r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_m_delta_r' not defined");
  }

  // n Params (z axis moment)
  if (!this->get_parameter("C_n_O", Cn_.O)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_n_O' not defined");
  }
  if (!this->get_parameter("C_n_alpha", Cn_.alpha)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_n_alpha' not defined");
  }
  if (!this->get_parameter("C_n_beta", Cn_.beta)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_n_beta' not defined");
  }
  if (!this->get_parameter("C_n_p", Cn_.p)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_n_p' not defined");
  }
  if (!this->get_parameter("C_n_q", Cn_.q)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_n_q' not defined");
  }
  if (!this->get_parameter("C_n_r", Cn_.r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_n_r' not defined");
  }
  if (!this->get_parameter("C_n_delta_a", Cn_.delta_a)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_n_delta_a' not defined");
  }
  if (!this->get_parameter("C_n_delta_e", Cn_.delta_e)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_n_delta_e' not defined");
  }
  if (!this->get_parameter("C_n_delta_r", Cn_.delta_r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_n_delta_r' not defined");
  }

  // Y Params (Sideslip Forces)
  if (!this->get_parameter("C_Y_O", CY_.O)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_Y_O' not defined");
  }
  if (!this->get_parameter("C_Y_alpha", CY_.alpha)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_Y_alpha' not defined");
  }
  if (!this->get_parameter("C_Y_beta", CY_.beta)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_Y_beta' not defined");
  }
  if (!this->get_parameter("C_Y_p", CY_.p)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_Y_p' not defined");
  }
  if (!this->get_parameter("C_Y_q", CY_.q)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_Y_q' not defined");
  }
  if (!this->get_parameter("C_Y_r", CY_.r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_Y_r' not defined");
  }
  if (!this->get_parameter("C_Y_delta_a", CY_.delta_a)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_Y_delta_a' not defined");
  }
  if (!this->get_parameter("C_Y_delta_e", CY_.delta_e)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_Y_delta_e' not defined");
  }
  if (!this->get_parameter("C_Y_delta_r", CY_.delta_r)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'C_Y_delta_r' not defined");
  }
}

rcl_interfaces::msg::SetParametersResult VTOL::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // Flag when a parameter is changed
  paramsHaveChanged_ = true;

  return result;
}

geometry_msgs::msg::WrenchStamped VTOL::update_forces_and_torques(rosflight_msgs::msg::SimState x,
                                                                        geometry_msgs::msg::Vector3Stamped wind,
                                                                        std::array<uint16_t, 14> act_cmds)
{
  // Update params if they have changed
  if (paramsHaveChanged_) {
    update_params_from_ROS();
    paramsHaveChanged_ = false;
  }

  // Multi-rotor-----------------------------------------------------------------------------------

  // Note that the motor commands should not be unmixed here for the multirotor aircraft, since the
  // values are already raw motor outputs.

  // // TODO: We should change this to the nonlinear model as a function of altitude
  // double rho = this->get_parameter("rho").as_double();

  // Get airspeed vector for drag force calculation (rotate wind into
  // body frame and subtract from inertial velocity in body frame)
  Eigen::Quaterniond q_body_to_inertial(x.pose.orientation.w,
                                        x.pose.orientation.x,
                                        x.pose.orientation.y,
                                        x.pose.orientation.z);
  Eigen::Vector3d v_wind_inertial(wind.vector.x, wind.vector.y, wind.vector.z); // In inertial frame
  Eigen::Vector3d v_wind_body = q_body_to_inertial.inverse() * v_wind_inertial; // Rotate to body frame

  // Compute airspeed
  // mr_Va = [ur, vr, wr] = V_ground - V_wind
  Eigen::Vector3d Vg(x.twist.linear.x, x.twist.linear.y, x.twist.linear.z); // Inertial velocities in body frame
  Eigen::Vector3d mr_Va = Vg - v_wind_body;

  // TODO: Would be better to compute the vector force/torque contributed by each motor,
  // not assume that the motors are aligned vertically
  double total_thrust = 0.0;
  double total_roll_torque = 0.0;
  double total_pitch_torque = 0.0;
  double total_yaw_torque = 0.0;

  // Convert PWM commands to [0,1]
  for (int i = 0; i < num_rotors_; i++) {
    double motor_cmd = (act_cmds[i] - 1000)/1000.0;

    // Saturate from 0 to 1 -- physical PWM constraints
    motor_cmd = sat(motor_cmd, 0, 1);

    motors_[i].command = motor_cmd;

  }

  for (MR_Motor motor : motors_) {
    // Calculate prop_speed
    // Use model with V_max
    double v_in = V_max_ * motor.command;

    // Take true angle and only compare the portion parallel to the body z-axis.
    // TODO: Add better model of propeller with non-straight aero forces
    Eigen::Vector3d body_z;
    body_z << 0.0, 0.0, 1.0;

    double Va_along_z = (-mr_Va).dot(body_z);

    // Saturate the airspeed to zero. The model therefore assumes that air flowing backwards
    // through the propeller has the same effect (on CT and CQ) as stationary air.
    if (Va_along_z < 0.0) {
      Va_along_z = 0.0;
    }

    double a = (motor.prop.CQ_0 * (rho_) * (pow((motor.prop.diam), 5.0)) / (pow((2 * M_PI), 2.0)));
    double b = (motor.prop.CQ_1 * (rho_) * (pow((motor.prop.diam), 4.0)) / (2 * M_PI)) * Va_along_z
      + ((pow((motor.KQ), 2.0)) / (motor.R));
    double c = (motor.prop.CQ_2 * (rho_) * (pow((motor.prop.diam), 3.0)) * (pow(Va_along_z, 2.0)))
      - ((motor.KQ) / (motor.R)) * v_in + ((motor.KQ) * (motor.I_0)); // COULD BE OUR CULPRIT

    double prop_speed = ((-b + sqrt((pow((b), 2.0)) - (4 * a * c))) / (2 * a));

    // Protect against singularities -- prop speed should be positive 
    if (prop_speed < 0.0001) {
      prop_speed = 0.0001;
    }

    // Calculate the advance ratio.
    double advance_ratio = (2. * M_PI * Va_along_z)/(prop_speed*motor.prop.diam);

    // Saturate the advance ratio at zero. The model therefore assumes that air flowing backwards
    // through the propeller has the same effect (on CT and CQ) as stationary air.
    if (advance_ratio < 0.0) {
      advance_ratio = 0.0;
    }

    double CT = motor.prop.CT_2*pow(advance_ratio, 2) + motor.prop.CT_1*advance_ratio + motor.prop.CT_0;
    if (CT <= 0.0) {
      CT = 0.0001;
    }

    double motor_thrust = CT * (rho_ * pow(motor.prop.diam, 4))/(4*pow(M_PI,2)) * pow(prop_speed, 2);

    double motor_roll_torque = -motor_thrust*motor.dist*sinf(motor.radial_angle);
    double motor_pitch_torque = motor_thrust*motor.dist*cosf(motor.radial_angle);

    double CQ = motor.prop.CQ_2*pow(advance_ratio, 2) + motor.prop.CQ_1*advance_ratio + motor.prop.CQ_0;
    if (CQ <= 0.0) {
      CQ = 0.0001;
    }

    // The torque produced by the propeller spinning.
    double prop_torque = motor.direction * CQ * (rho_*pow(motor.prop.diam, 5))/(4*pow(M_PI,2)) * pow(prop_speed, 2);

    total_thrust += motor_thrust;
    total_roll_torque += motor_roll_torque;
    total_pitch_torque += motor_pitch_torque;
    total_yaw_torque += prop_torque;
  }

  // Compute aerodynamic drag forces on the body of the vehicle
  double cross_sectional_area = this->get_parameter("A_c").as_double(); // m^2
  double CD = this->get_parameter("CD").as_double();

  double drag = 0.5 * rho_ * cross_sectional_area * CD * pow(mr_Va.norm(), 2);
  Eigen::Vector3d drag_force;

  if (mr_Va.norm() < 0.001) {
    drag_force << 0.0, 0.0, 0.0;
  } else {
    drag_force = -drag * mr_Va / mr_Va.norm();
  }

  // Compute the drag force induced from the propellers
  double CD_induced = this->get_parameter("CD_induced").as_double();

  Eigen::Matrix3d drag_matrix;
  drag_matrix << -total_thrust*CD_induced, 0.0, 0.0,
                 0.0, -total_thrust*CD_induced, 0.0,
                 0.0, 0.0, 0.0;

  Eigen::Vector3d induced_drag_force = drag_matrix * mr_Va;

  // Apply Ground Effect
  std::vector<double> ground_effect_coeffs = this->get_parameter("ground_effect").as_double_array();
  double ground_effect_force = max(0.0, ground_effect_coeffs[0]*pow(-x.pose.position.z, 4) +
                                        ground_effect_coeffs[1]*pow(-x.pose.position.z, 3) +
                                        ground_effect_coeffs[2]*pow(-x.pose.position.z, 2) +
                                        ground_effect_coeffs[3]*-x.pose.position.z +
                                        ground_effect_coeffs[4]);
  Eigen::Vector3d ground_effect(0.0, 0.0, -ground_effect_force);


  // Add the forces together in the body frame
  Eigen::Vector3d mr_body_forces;
  mr_body_forces << 0.0, 0.0, -total_thrust; // negative Z direction
  mr_body_forces += drag_force;
  mr_body_forces += induced_drag_force;
  mr_body_forces += ground_effect;

  Eigen::Vector3d mr_body_torques;
  mr_body_torques << total_roll_torque, total_pitch_torque, -total_yaw_torque;

  // Fixed-wing-----------------------------------------------------------------------------------------------------

  // TODO: The mixer used is not either of the vanilla PRIMARY or SECONDARY mixers... It is a combination,
  // based on the current value of the RC_OVERRIDE switch... I need to add that logic here.
  // This will need to be done after the RC override switch is updated to reflect attitude vs throttle override (separate Github issue)
  // Until then, this simulator will always use the primary mixer (not any combo of primary and secondary that is possible in firmware).

  // Convert PWM into 0 to 1 output - note that the mixer does not touch the last (NUM_TOTAL_OUTPUTS - NUM_MIXER_OUTPUTS)
  // number of outputs
  Eigen::Matrix<double, NUM_MIXER_OUTPUTS, 1> act_cmds_vect;
  for (int i=0; i<NUM_MIXER_OUTPUTS; ++i) {
    // {0, 1, 2, 3} = {aux, servo, motor, gpio}
    switch (mixer_header_vals_(i)) {
      case 0:
      case 4:
        act_cmds_vect(i) = static_cast<double>(act_cmds[i]);
        break;
      case 1:
        act_cmds_vect(i) = (static_cast<double>(act_cmds[i]) - 1500.0) / 500.0;
        break;
      case 2:
        act_cmds_vect(i) = (static_cast<double>(act_cmds[i]) - 1000.0) / 1000.0;
        break;
      default:
        RCLCPP_ERROR_STREAM(this->get_logger(), "Mixer header has unknown value!");
        act_cmds_vect(i) = 0.0;
        break;
    }
  }

  // Unmix the actuator commands -- Note that the primary_mixing_matrix_ written here is
  // M, not M^{-1} as defined in Small Unmanned Aircraft, Ch. 14
  Eigen::VectorXd act_cmds_unmixed = primary_mixing_matrix_ * act_cmds_vect;

  // Act cmds unmixed are {Fx, Fy, Fz, Qx, Qy, Qz}
  // Scale the unmixed commands by the maximum control surface deflection angle
  // TODO: Does doing it this way make sense for a vtail configuration?
  delta_.a = act_cmds_unmixed(3) * this->get_parameter("max_aileron_deflection_angle").as_double() * M_PI / 180.0;
  delta_.e = act_cmds_unmixed(4) * this->get_parameter("max_elevator_deflection_angle").as_double() * M_PI / 180.0;;
  delta_.t = act_cmds_unmixed(0);
  delta_.r = act_cmds_unmixed(5) * this->get_parameter("max_rudder_deflection_angle").as_double() * M_PI / 180.0;;

  // Apply servo time delay
  Actuators delta_curr;

  float Ts = this->get_parameter("servo_refresh_rate").as_double(); // refresh rate TODO: find a way to programmatically set this.
  delta_curr.a = 1 / (2 * servo_tau_ / Ts + 1)
    * (delta_prev_command_.a + delta_.a + (2 * servo_tau_ / Ts - 1) * delta_prev_.a);
  delta_curr.e = 1 / (2 * servo_tau_ / Ts + 1)
    * (delta_prev_command_.e + delta_.e + (2 * servo_tau_ / Ts - 1) * delta_prev_.e);
  delta_curr.r = 1 / (2 * servo_tau_ / Ts + 1)
    * (delta_prev_command_.r + delta_.r + (2 * servo_tau_ / Ts - 1) * delta_prev_.r);

  delta_prev_ = delta_curr;
  delta_prev_command_ = delta_;

  double p = x.twist.angular.x;
  double q = x.twist.angular.y;
  double r = x.twist.angular.z;

  // Calculate airspeed
  Eigen::Quaterniond q_body_to_inertial(x.pose.orientation.w,
                                        x.pose.orientation.x,
                                        x.pose.orientation.y,
                                        x.pose.orientation.z);
  Eigen::Vector3d v_wind_inertial(wind.vector.x, wind.vector.y, wind.vector.z); // In inertial frame
  Eigen::Vector3d v_wind_body = q_body_to_inertial.inverse() * v_wind_inertial; // Rotate to body frame

  // Va = [ur, vr, wr] = V_ground - V_wind
  double ur = x.twist.linear.x - v_wind_body(0);
  double vr = x.twist.linear.y - v_wind_body(1);
  double wr = x.twist.linear.z - v_wind_body(2);

  double fw_Va = sqrt(pow(ur,2.0) + pow(vr,2.0) + pow(wr,2.0));

  geometry_msgs::msg::WrenchStamped forces;

  /*
   * The following math follows the method described in chapter 4 of
   * Small Unmanned Aircraft: Theory and Practice
   * By Randy Beard and Tim McLain.
   * Look there for a detailed explanation of each line in the rest of this function
   */

  double v_in = fw_motor_.V_max * delta_.t;

  double a = (fw_prop_.CQ_0 * (rho_) * (pow((fw_prop_.D_prop), 5.0)) / (pow((2 * M_PI), 2.0)));
  double b = (fw_prop_.CQ_1 * (rho_) * (pow((fw_prop_.D_prop), 4.0)) / (2 * M_PI)) * fw_Va
    + ((pow((fw_motor_.KQ), 2.0)) / (fw_motor_.R_motor));
  double c = (fw_prop_.CQ_2 * (rho_) * (pow((fw_prop_.D_prop), 3.0)) * (pow((fw_Va), 2.0)))
    - ((fw_motor_.KQ) / (fw_motor_.R_motor)) * v_in + ((fw_motor_.KQ) * (fw_motor_.I_0));

  double Omega_p = ((-b + sqrt((pow((b), 2.0)) - (4 * a * c))) / (2 * a));

  double Prop_Force = ((rho_) * (pow((fw_prop_.D_prop), 4.0))
                       * (((fw_prop_.CT_0) * (pow((Omega_p), 2.0))) / (4 * (pow((M_PI), 2.0)))))
    + ((rho_) * (pow((fw_prop_.D_prop), 3.0)) * (fw_prop_.CT_1) * (fw_Va) * (Omega_p) / (2 * M_PI))
    + ((rho_) * (pow((fw_prop_.D_prop), 2.0)) * (fw_prop_.CT_2) * (pow((fw_Va), 2.0)));

  double Prop_Torque = ((rho_) * (pow((fw_prop_.D_prop), 5.0))
                        * ((((fw_prop_.CQ_0) / (4 * (pow((M_PI), 2.0))) * (pow((Omega_p), 2.0))))))
    + ((rho_) * (pow((fw_prop_.D_prop), 4.0)) * (fw_prop_.CQ_1) * (fw_Va) * (Omega_p) / (2 * M_PI))
    + ((rho_) * (pow((fw_prop_.D_prop), 3.0)) * (fw_prop_.CQ_2) * (pow((fw_Va), 2.0)));

  // Be sure that we have some significant airspeed before we run aerodynamics, and don't let NaNs get through
  if (fw_Va > 2.0 && std::isfinite(fw_Va)) {
    double alpha = atan2(wr, ur);
    double beta = asin(vr / fw_Va);

    double ca = cos(alpha);
    double sa = sin(alpha);

    double sign = (alpha >= 0 ? 1 : -1); // Sigmoid function
    double sigma_a = 0.0;
    //  (1 + exp(-(wing_.M * (alpha - wing_.alpha0))) + exp((wing_.M * (alpha + wing_.alpha0))))
    //  / ((1 + exp(-(wing_.M * (alpha - wing_.alpha0))))
    //     * (1 + exp((wing_.M * (alpha + wing_.alpha0)))));
    double CL_a = (1 - sigma_a) * (CL_.O + CL_.alpha * alpha) + sigma_a * (2 * sign * sa * sa * ca);
    double AR = (pow(wing_.b, 2.0)) / wing_.S;
    double CD_a = CD_.p + ((pow((CL_.O + CL_.alpha * (alpha)), 2.0)) / (3.14159 * 0.9 * AR));
    // the const 0.9 in this equation replaces the e (Oswald Factor) variable and may be inaccurate

    double CX_a = -CD_a * ca + CL_a * sa;
    double CX_q_a = -CD_.q * ca + CL_.q * sa;
    double CX_deltaE_a = -CD_.delta_e * ca + CL_.delta_e * sa;

    double CZ_a = -CD_a * sa - CL_a * ca;
    double CZ_q_a = -CD_.q * sa - CL_.q * ca;
    double CZ_deltaE_a = -CD_.delta_e * sa - CL_.delta_e * ca;

    forces.wrench.force.x = 0.5 * (rho_) *fw_Va * fw_Va * wing_.S
        * (CX_a + (CX_q_a * wing_.c * q) / (2.0 * fw_Va) + CX_deltaE_a * delta_curr.e)
      + Prop_Force;

    forces.wrench.force.y = 0.5 * (rho_) *fw_Va * fw_Va * wing_.S
      * (CY_.O + CY_.beta * beta + ((CY_.p * wing_.b * p) / (2.0 * fw_Va))
         + ((CY_.r * wing_.b * r) / (2.0 * fw_Va)) + CY_.delta_a * delta_curr.a
         + CY_.delta_r * delta_.r);

    forces.wrench.force.z = 0.5 * (rho_) *fw_Va * fw_Va * wing_.S
      * (CZ_a + (CZ_q_a * wing_.c * q) / (2.0 * fw_Va) + CZ_deltaE_a * delta_curr.e);

    forces.wrench.torque.x = 0.5 * (rho_) *fw_Va * fw_Va * wing_.S * wing_.b
        * (Cell_.O + Cell_.beta * beta + (Cell_.p * wing_.b * p) / (2.0 * fw_Va)
           + (Cell_.r * wing_.b * r) / (2.0 * fw_Va) + Cell_.delta_a * delta_curr.a
           + Cell_.delta_r * delta_.r)
      - Prop_Torque;

    forces.wrench.torque.y = 0.5 * (rho_) *fw_Va * fw_Va * wing_.S * wing_.c
      * (Cm_.O + Cm_.alpha * alpha + (Cm_.q * wing_.c * q) / (2.0 * fw_Va)
         + Cm_.delta_e * delta_curr.e);

    forces.wrench.torque.z = 0.5 * (rho_) *fw_Va * fw_Va * wing_.S * wing_.b
      * (Cn_.O + Cn_.beta * beta + (Cn_.p * wing_.b * p) / (2.0 * fw_Va)
         + (Cn_.r * wing_.b * r) / (2.0 * fw_Va) + Cn_.delta_a * delta_curr.a
         + Cn_.delta_r * delta_.r);
  } else {

    if (delta_.t < 0.01) {
      forces.wrench.force.x = 0.0;
    } else {
      forces.wrench.force.x = Prop_Force;
    }

    forces.wrench.force.y = 0.0;
    forces.wrench.force.z = 0.0;
    // We do not simulate torque in the low speed conditions.
    // This is because it usually means we are on the ground.
    forces.wrench.torque.x = 0.0;
    forces.wrench.torque.y = 0.0;
    forces.wrench.torque.z = 0.0;
  }

  // Package up the message and return it
  rclcpp::Time now = this->get_clock()->now();
  forces.header.stamp = now;

  return forces;


  // Package up message and return
  geometry_msgs::msg::WrenchStamped msg;

  msg.header.stamp = this->get_clock()->now();

  msg.wrench.force.x = mr_body_forces(0);
  msg.wrench.force.y = mr_body_forces(1);
  msg.wrench.force.z = mr_body_forces(2);
  msg.wrench.torque.x = mr_body_torques(0);
  msg.wrench.torque.y = mr_body_torques(1);
  msg.wrench.torque.z = mr_body_torques(2);

  return msg;
}

void VTOL::get_firmware_parameters()
{
  // No extra parameters to query since mixer has already been queried
}

} // namespace rosflight_sim


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosflight_sim::VTOL>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
