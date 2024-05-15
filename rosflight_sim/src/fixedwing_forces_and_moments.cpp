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

#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rosflight_sim/fixedwing_forces_and_moments.hpp>

namespace rosflight_sim
{
Fixedwing::Fixedwing(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
    , rho_(0)
    , wing_()
    , prop_()
    , motor_()
    , CL_()
    , CD_()
    , Cm_()
    , CY_()
    , Cell_()
    , Cn_()
    , delta_()
{
  declare_fixedwing_params();
  update_params_from_ROS();
  wind_ = Eigen::Vector3d::Zero();
  forces_moments_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/forces_and_moments", 1);
}

Fixedwing::~Fixedwing() = default;

void Fixedwing::declare_fixedwing_params()
{
  node_->declare_parameter("rho", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("wing_s", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("wing_b", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("wing_c", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("wing_M", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("wing_epsilon", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("wing_alpha0", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("servo_tau", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_L_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_delta_r", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_D_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_delta_r", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_ell_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_delta_r", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_m_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_delta_r", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_n_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_delta_r", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_Y_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_delta_r", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("D_prop", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("CT_0", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("CT_1", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("CT_2", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("CQ_0", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("CQ_1", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("CQ_2", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("KV", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("KQ", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("V_max", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("R_motor", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("I_0", rclcpp::PARAMETER_DOUBLE);

}

void Fixedwing::update_params_from_ROS()
{
  if (!node_->get_parameter("rho", rho_)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'rho' not defined");
  }

  // Wing Geometry
  if (!node_->get_parameter("wing_s", wing_.S)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'wing_s' not defined");
  }
  if (!node_->get_parameter("wing_b", wing_.b)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'wing_b' not defined");
  }
  if (!node_->get_parameter("wing_c", wing_.c)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'wing_c' not defined");
  }
  if (!node_->get_parameter("wing_M", wing_.M)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'wing_M' not defined");
  }
  if (!node_->get_parameter("wing_epsilon", wing_.epsilon)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'wing_epsilon' not defined");
  }
  if (!node_->get_parameter("wing_alpha0", wing_.alpha0)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'wing_alpha0' not defined");
  }

  // Propeller Coefficients
  if (!node_->get_parameter("D_prop", prop_.D_prop)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'D_prop' not defined");
  }
  if (!node_->get_parameter("CT_0", prop_.CT_0)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'CT_0' not defined");
  }
  if (!node_->get_parameter("CT_1", prop_.CT_1)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'CT_1' not defined");
  }
  if (!node_->get_parameter("CT_2", prop_.CT_2)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'CT_2' not defined");
  }
  if (!node_->get_parameter("CQ_0", prop_.CQ_0)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'CQ_0' not defined");
  }
  if (!node_->get_parameter("CQ_1", prop_.CQ_1)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'CQ_1' not defined");
  }
  if (!node_->get_parameter("CQ_2", prop_.CQ_2)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'CQ_2' not defined");
  }

  // Motor Coefficients
  if (!node_->get_parameter("KV", motor_.KV)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'KV' not defined");
  }
  if (!node_->get_parameter("KQ", motor_.KQ)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'KQ' not defined");
  }
  if (!node_->get_parameter("V_max", motor_.V_max)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'V_max' not defined");
  }
  if (!node_->get_parameter("R_motor", motor_.R_motor)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'R_motor' not defined");
  }
  if (!node_->get_parameter("I_0", motor_.I_0)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'I_0' not defined");
  }

  if (!node_->get_parameter("servo_tau", servo_tau_)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'servo_tau' not defined");
  }

  // Lift Params
  if (!node_->get_parameter("C_L_O", CL_.O)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_L_O' not defined");
  }
  if (!node_->get_parameter("C_L_alpha", CL_.alpha)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_L_alpha' not defined");
  }
  if (!node_->get_parameter("C_L_beta", CL_.beta)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_L_beta' not defined");
  }
  if (!node_->get_parameter("C_L_p", CL_.p)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_L_p' not defined");
  }
  if (!node_->get_parameter("C_L_q", CL_.q)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_L_q' not defined");
  }
  if (!node_->get_parameter("C_L_r", CL_.r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_L_r' not defined");
  }
  if (!node_->get_parameter("C_L_delta_a", CL_.delta_a)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_L_delta_a' not defined");
  }
  if (!node_->get_parameter("C_L_delta_e", CL_.delta_e)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_L_delta_e' not defined");
  }
  if (!node_->get_parameter("C_L_delta_r", CL_.delta_r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_L_delta_r' not defined");
  }

  // Drag Params
  if (!node_->get_parameter("C_D_O", CD_.O)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_D_O' not defined");
  }
  if (!node_->get_parameter("C_D_alpha", CD_.alpha)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_D_alpha' not defined");
  }
  if (!node_->get_parameter("C_D_beta", CD_.beta)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_D_beta' not defined");
  }
  if (!node_->get_parameter("C_D_p", CD_.p)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_D_p' not defined");
  }
  if (!node_->get_parameter("C_D_q", CD_.q)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_D_q' not defined");
  }
  if (!node_->get_parameter("C_D_r", CD_.r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_D_r' not defined");
  }
  if (!node_->get_parameter("C_D_delta_a", CD_.delta_a)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_D_delta_a' not defined");
  }
  if (!node_->get_parameter("C_D_delta_e", CD_.delta_e)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_D_delta_e' not defined");
  }
  if (!node_->get_parameter("C_D_delta_r", CD_.delta_r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_D_delta_r' not defined");
  }

  // ell Params (x axis moment)
  if (!node_->get_parameter("C_ell_O", Cell_.O)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_ell_O' not defined");
  }
  if (!node_->get_parameter("C_ell_alpha", Cell_.alpha)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_ell_alpha' not defined");
  }
  if (!node_->get_parameter("C_ell_beta", Cell_.beta)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_ell_beta' not defined");
  }
  if (!node_->get_parameter("C_ell_p", Cell_.p)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_ell_p' not defined");
  }
  if (!node_->get_parameter("C_ell_q", Cell_.q)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_ell_q' not defined");
  }
  if (!node_->get_parameter("C_ell_r", Cell_.r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_ell_r' not defined");
  }
  if (!node_->get_parameter("C_ell_delta_a", Cell_.delta_a)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_ell_delta_a' not defined");
  }
  if (!node_->get_parameter("C_ell_delta_e", Cell_.delta_e)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_ell_delta_e' not defined");
  }
  if (!node_->get_parameter("C_ell_delta_r", Cell_.delta_r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_ell_delta_r' not defined");
  }

  // m Params (y axis moment)
  if (!node_->get_parameter("C_m_O", Cm_.O)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_m_O' not defined");
  }
  if (!node_->get_parameter("C_m_alpha", Cm_.alpha)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_m_alpha' not defined");
  }
  if (!node_->get_parameter("C_m_beta", Cm_.beta)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_m_beta' not defined");
  }
  if (!node_->get_parameter("C_m_p", Cm_.p)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_m_p' not defined");
  }
  if (!node_->get_parameter("C_m_q", Cm_.q)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_m_q' not defined");
  }
  if (!node_->get_parameter("C_m_r", Cm_.r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_m_r' not defined");
  }
  if (!node_->get_parameter("C_m_delta_a", Cm_.delta_a)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_m_delta_a' not defined");
  }
  if (!node_->get_parameter("C_m_delta_e", Cm_.delta_e)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_m_delta_e' not defined");
  }
  if (!node_->get_parameter("C_m_delta_r", Cm_.delta_r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_m_delta_r' not defined");
  }

  // n Params (z axis moment)
  if (!node_->get_parameter("C_n_O", Cn_.O)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_n_O' not defined");
  }
  if (!node_->get_parameter("C_n_alpha", Cn_.alpha)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_n_alpha' not defined");
  }
  if (!node_->get_parameter("C_n_beta", Cn_.beta)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_n_beta' not defined");
  }
  if (!node_->get_parameter("C_n_p", Cn_.p)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_n_p' not defined");
  }
  if (!node_->get_parameter("C_n_q", Cn_.q)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_n_q' not defined");
  }
  if (!node_->get_parameter("C_n_r", Cn_.r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_n_r' not defined");
  }
  if (!node_->get_parameter("C_n_delta_a", Cn_.delta_a)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_n_delta_a' not defined");
  }
  if (!node_->get_parameter("C_n_delta_e", Cn_.delta_e)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_n_delta_e' not defined");
  }
  if (!node_->get_parameter("C_n_delta_r", Cn_.delta_r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_n_delta_r' not defined");
  }

  // Y Params (Sideslip Forces)
  if (!node_->get_parameter("C_Y_O", CY_.O)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_Y_O' not defined");
  }
  if (!node_->get_parameter("C_Y_alpha", CY_.alpha)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_Y_alpha' not defined");
  }
  if (!node_->get_parameter("C_Y_beta", CY_.beta)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_Y_beta' not defined");
  }
  if (!node_->get_parameter("C_Y_p", CY_.p)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_Y_p' not defined");
  }
  if (!node_->get_parameter("C_Y_q", CY_.q)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_Y_q' not defined");
  }
  if (!node_->get_parameter("C_Y_r", CY_.r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_Y_r' not defined");
  }
  if (!node_->get_parameter("C_Y_delta_a", CY_.delta_a)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_Y_delta_a' not defined");
  }
  if (!node_->get_parameter("C_Y_delta_e", CY_.delta_e)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_Y_delta_e' not defined");
  }
  if (!node_->get_parameter("C_Y_delta_r", CY_.delta_r)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'C_Y_delta_r' not defined");
  }
}

Eigen::Matrix<double, 6, 1> Fixedwing::update_forces_and_torques(CurrentState x,
                                                                 const int act_cmds[])
{
  delta_.a = (act_cmds[0] - 1500.0) / 500.0;
  delta_.e = -(act_cmds[1] - 1500.0) / 500.0;
  delta_.t = (act_cmds[2] - 1000.0) / 1000.0;
  delta_.r = -(act_cmds[3] - 1500.0) / 500.0;

  Actuators delta_curr;

  float Ts = .003; // refresh rate TODO find a way to programmatically set this.

  delta_curr.a = 1 / (2 * servo_tau_ / Ts + 1)
    * (delta_prev_command_.a + delta_.a + (2 * servo_tau_ / Ts - 1) * delta_prev_.a);
  delta_curr.e = 1 / (2 * servo_tau_ / Ts + 1)
    * (delta_prev_command_.e + delta_.e + (2 * servo_tau_ / Ts - 1) * delta_prev_.e);

  delta_prev_ = delta_curr;

  delta_prev_command_ = delta_;

  double p = x.omega(0);
  double q = x.omega(1);
  double r = x.omega(2);

  // Calculate airspeed
  Eigen::Vector3d V_airspeed = x.vel + x.rot.inverse() * wind_;
  double ur = V_airspeed(0);
  double vr = V_airspeed(1);
  double wr = V_airspeed(2);

  double Va = V_airspeed.norm();

  Eigen::Matrix<double, 6, 1> forces;

  /*
   * The following math follows the method described in chapter 4 of
   * Small Unmanned Aircraft: Theory and Practice
   * By Randy Beard and Tim McLain.
   * Look there for a detailed explanation of each line in the rest of this function
   */

  double v_in = motor_.V_max * delta_.t;

  double a = (prop_.CQ_0 *(rho_) *(pow((prop_.D_prop), 5.0)) / (pow((2 * M_PI), 2.0)));
  double b = (prop_.CQ_1 *(rho_) *(pow((prop_.D_prop), 4.0)) / (2 * M_PI)) * Va 
            + ((pow((motor_.KQ), 2.0)) /(motor_.R_motor));
  double c = (prop_.CQ_2 *(rho_) *(pow((prop_.D_prop), 3.0)) * (pow((Va), 2.0))) 
              - ((motor_.KQ)/(motor_.R_motor)) *v_in + ((motor_.KQ) *(motor_.I_0));

  double Omega_p = ((-b + sqrt((pow((b), 2.0)) - (4 *a *c))) / (2 *a));

  double Prop_Force = ((rho_) *(pow((prop_.D_prop), 4.0)) *(((prop_.CT_0) *(pow((Omega_p), 2.0)))/ (4*(pow((M_PI), 2.0))))) 
              + ((rho_) *(pow((prop_.D_prop), 3.0)) *(prop_.CT_1) *(Va) *(Omega_p)/(2 * M_PI)) 
              + ((rho_) *(pow((prop_.D_prop), 2.0)) *(prop_.CT_2) *(pow((Va), 2.0)));

  double Prop_Torque = ((rho_) *(pow((prop_.D_prop), 5.0)) *((((prop_.CQ_0)/(4 *(pow((M_PI), 2.0))) *(pow((Omega_p), 2.0))))))
              + ((rho_) *(pow((prop_.D_prop), 4.0)) *(prop_.CQ_1) *(Va) *(Omega_p)/(2 * M_PI))
              + ((rho_) *(pow((prop_.D_prop), 3.0)) *(prop_.CQ_2) *(pow((Va), 2.0)));

  // Be sure that we have some significant airspeed before we run aerodynamics, and don't let NaNs get through
  if (Va > 1.0 && std::isfinite(Va)) {
    double alpha = atan2(wr, ur);
    double beta = asin(vr / Va);

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

    forces(0) = 0.5 * (rho_) *Va * Va * wing_.S
        * (CX_a + (CX_q_a * wing_.c * q) / (2.0 * Va) + CX_deltaE_a * delta_curr.e)
      + Prop_Force;

    forces(1) = 0.5 * (rho_) *Va * Va * wing_.S
      * (CY_.O + CY_.beta * beta + ((CY_.p * wing_.b * p) / (2.0 * Va))
         + ((CY_.r * wing_.b * r) / (2.0 * Va)) + CY_.delta_a * delta_curr.a
         + CY_.delta_r * delta_.r);

    forces(2) = 0.5 * (rho_) *Va * Va * wing_.S
      * (CZ_a + (CZ_q_a * wing_.c * q) / (2.0 * Va) + CZ_deltaE_a * delta_curr.e);

    forces(3) = 0.5 * (rho_) *Va * Va * wing_.S * wing_.b
        * (Cell_.O + Cell_.beta * beta + (Cell_.p * wing_.b * p) / (2.0 * Va)
           + (Cell_.r * wing_.b * r) / (2.0 * Va) + Cell_.delta_a * delta_curr.a
           + Cell_.delta_r * delta_.r) - Prop_Torque;

    forces(4) = 0.5 * (rho_) *Va * Va * wing_.S * wing_.c
      * (Cm_.O + Cm_.alpha * alpha + (Cm_.q * wing_.c * q) / (2.0 * Va)
         + Cm_.delta_e * delta_curr.e);

    forces(5) = 0.5 * (rho_) *Va * Va * wing_.S * wing_.b
      * (Cn_.O + Cn_.beta * beta + (Cn_.p * wing_.b * p) / (2.0 * Va)
         + (Cn_.r * wing_.b * r) / (2.0 * Va) + Cn_.delta_a * delta_curr.a
         + Cn_.delta_r * delta_.r);
  } else {
  
    if (delta_.t == 0.0) {
      forces(0) = 0.0;
    }
    else {
      forces(0) = Prop_Force;
    }
    
    forces(1) = 0.0;
    forces(2) = 0.0;
    forces(3) = 0.0; // We do not simulate torque in the low speed conditions, this is because it usually means we are on the ground.
    forces(4) = 0.0;
    forces(5) = 0.0;
  }

  geometry_msgs::msg::TwistStamped msg;
  
  rclcpp::Time now = node_->get_clock()->now();
  
  msg.header.stamp = now;

  msg.twist.linear.x = forces(0);
  msg.twist.linear.y = forces(1);
  msg.twist.linear.z = forces(2);

  msg.twist.angular.x = forces(3);
  msg.twist.angular.y = forces(4);
  msg.twist.angular.z = forces(5);

  forces_moments_pub_->publish(msg);

  return forces;
}

void Fixedwing::set_wind(Eigen::Vector3d wind) { wind_ = wind; }

} // namespace rosflight_sim
