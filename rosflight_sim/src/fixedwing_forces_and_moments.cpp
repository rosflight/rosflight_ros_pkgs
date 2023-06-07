/*
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
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

#include <rosflight_sim/fixedwing_forces_and_moments.h>

namespace rosflight_sim
{
Fixedwing::Fixedwing(rclcpp::Node::SharedPtr node) : node_(std::move(node))
{
  // physical parameters
  mass_ = node_->get_parameter_or("mass", 13.5);
  Jx_ = node_->get_parameter_or("Jx", 0.8244);
  Jy_ = node_->get_parameter_or("Jy", 1.135);
  Jz_ = node_->get_parameter_or("Jz", 1.759);
  Jxz_ = node_->get_parameter_or("Jxz", .1204);
  rho_ = node_->get_parameter_or("rho", 1.2682);

  // Wing Geometry
  wing_.S = node_->get_parameter_or("wing_s", 0.55);
  wing_.b = node_->get_parameter_or("wing_b", 2.8956);
  wing_.c = node_->get_parameter_or("wing_c", 0.18994);
  wing_.M = node_->get_parameter_or("wing_M", 0.55);
  wing_.epsilon = node_->get_parameter_or("wing_epsilon", 2.8956);
  wing_.alpha0 = node_->get_parameter_or("wing_alpha0", 0.18994);

  // Propeller Coefficients
  prop_.k_motor = node_->get_parameter_or("k_motor", 80.0);
  prop_.k_T_P = node_->get_parameter_or("k_T_P", 0.0);
  prop_.k_Omega = node_->get_parameter_or("k_Omega", 0.0);
  prop_.e = node_->get_parameter_or("prop_e", 0.9);
  prop_.S = node_->get_parameter_or("prop_S", 0.202);
  prop_.C = node_->get_parameter_or("prop_C", 1.0);

  // Lift Params
  CL_.O = node_->get_parameter_or("C_L_O", 0.28);
  CL_.alpha = node_->get_parameter_or("C_L_alpha", 3.45);
  CL_.beta = node_->get_parameter_or("C_L_beta", 0.0);
  CL_.p = node_->get_parameter_or("C_L_p", 0.0);
  CL_.q = node_->get_parameter_or("C_L_q", 0.0);
  CL_.r = node_->get_parameter_or("C_L_r", 0.0);
  CL_.delta_a = node_->get_parameter_or("C_L_delta_a", 0.0);
  CL_.delta_e = node_->get_parameter_or("C_L_delta_e", -0.36);
  CL_.delta_r = node_->get_parameter_or("C_L_delta_r", 0.0);

  // Drag Params
  CD_.O = node_->get_parameter_or("C_D_O", 0.03);
  CD_.alpha = node_->get_parameter_or("C_D_alpha", 0.30);
  CD_.beta = node_->get_parameter_or("C_D_beta", 0.0);
  CD_.p = node_->get_parameter_or("C_D_p", 0.0437);
  CD_.q = node_->get_parameter_or("C_D_q", 0.0);
  CD_.r = node_->get_parameter_or("C_D_r", 0.0);
  CD_.delta_a = node_->get_parameter_or("C_D_delta_a", 0.0);
  CD_.delta_e = node_->get_parameter_or("C_D_delta_e", 0.0);
  CD_.delta_r = node_->get_parameter_or("C_D_delta_r", 0.0);

  // ell Params (x axis moment)
  Cell_.O = node_->get_parameter_or("C_ell_O", 0.0);
  Cell_.alpha = node_->get_parameter_or("C_ell_alpha", 0.00);
  Cell_.beta = node_->get_parameter_or("C_ell_beta", -0.12);
  Cell_.p = node_->get_parameter_or("C_ell_p", -0.26);
  Cell_.q = node_->get_parameter_or("C_ell_q", 0.0);
  Cell_.r = node_->get_parameter_or("C_ell_r", 0.14);
  Cell_.delta_a = node_->get_parameter_or("C_ell_delta_a", 0.08);
  Cell_.delta_e = node_->get_parameter_or("C_ell_delta_e", 0.0);
  Cell_.delta_r = node_->get_parameter_or("C_ell_delta_r", 0.105);

  // m Params (y axis moment)
  Cm_.O = node_->get_parameter_or("C_m_O", -0.02338);
  Cm_.alpha = node_->get_parameter_or("C_m_alpha", -0.38);
  Cm_.beta = node_->get_parameter_or("C_m_beta", 0.0);
  Cm_.p = node_->get_parameter_or("C_m_p", 0.0);
  Cm_.q = node_->get_parameter_or("C_m_q", -3.6);
  Cm_.r = node_->get_parameter_or("C_m_r", 0.0);
  Cm_.delta_a = node_->get_parameter_or("C_m_delta_a", 0.0);
  Cm_.delta_e = node_->get_parameter_or("C_m_delta_e", -0.5);
  Cm_.delta_r = node_->get_parameter_or("C_m_delta_r", 0.0);

  // n Params (z axis moment)
  Cn_.O = node_->get_parameter_or("C_n_O", 0.0);
  Cn_.alpha = node_->get_parameter_or("C_n_alpha", 0.0);
  Cn_.beta = node_->get_parameter_or("C_n_beta", 0.25);
  Cn_.p = node_->get_parameter_or("C_n_p", 0.022);
  Cn_.q = node_->get_parameter_or("C_n_q", 0.0);
  Cn_.r = node_->get_parameter_or("C_n_r", -0.35);
  Cn_.delta_a = node_->get_parameter_or("C_n_delta_a", 0.06);
  Cn_.delta_e = node_->get_parameter_or("C_n_delta_e", 0.0);
  Cn_.delta_r = node_->get_parameter_or("C_n_delta_r", -0.032);

  // Y Params (Sideslip Forces)
  CY_.O = node_->get_parameter_or("C_Y_O", 0.0);
  CY_.alpha = node_->get_parameter_or("C_Y_alpha", 0.00);
  CY_.beta = node_->get_parameter_or("C_Y_beta", -0.98);
  CY_.p = node_->get_parameter_or("C_Y_p", 0.0);
  CY_.q = node_->get_parameter_or("C_Y_q", 0.0);
  CY_.r = node_->get_parameter_or("C_Y_r", 0.0);
  CY_.delta_a = node_->get_parameter_or("C_Y_delta_a", 0.0);
  CY_.delta_e = node_->get_parameter_or("C_Y_delta_e", 0.0);
  CY_.delta_r = node_->get_parameter_or("C_Y_delta_r", -0.017);

  wind_ = Eigen::Vector3d::Zero();
}

Eigen::Matrix<double, 6, 1> Fixedwing::updateForcesAndTorques(Current_State x, const int act_cmds[])
{
  delta_.a = (act_cmds[0] - 1500.0) / 500.0;
  delta_.e = -(act_cmds[1] - 1500.0) / 500.0;
  delta_.t = (act_cmds[2] - 1000.0) / 1000.0;
  delta_.r = -(act_cmds[3] - 1500.0) / 500.0;

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

  // Be sure that we have some significant airspeed before we run aerodynamics, and don't let NaNs get through
  if (Va > 1.0 && std::isfinite(Va))
  {
    /*
     * The following math follows the method described in chapter 4 of
     * Small Unmanned Aircraft: Theory and Practice
     * By Randy Beard and Tim McLain.
     * Look there for a detailed explanation of each line in the rest of this function
     */
    double alpha = atan2(wr, ur);
    double beta = asin(vr / Va);

    double ca = cos(alpha);
    double sa = sin(alpha);

    double sign = (alpha >= 0 ? 1 : -1); // Sigmoid function
    double sigma_a = (1 + exp(-(wing_.M * (alpha - wing_.alpha0))) + exp((wing_.M * (alpha + wing_.alpha0))))
                     / ((1 + exp(-(wing_.M * (alpha - wing_.alpha0)))) * (1 + exp((wing_.M * (alpha + wing_.alpha0)))));
    double CL_a = (1 - sigma_a) * (CL_.O + CL_.alpha * alpha) + sigma_a * (2 * sign * sa * sa * ca);
    double AR = (pow(wing_.b, 2.0)) / wing_.S;
    double CD_a =
        CD_.p
        + ((pow((CL_.O + CL_.alpha * (alpha)), 2.0))
           / (3.14159 * 0.9
              * AR)); // the const 0.9 in this equation replaces the e (Oswald Factor) variable and may be inaccurate

    double CX_a = -CD_a * ca + CL_a * sa;
    double CX_q_a = -CD_.q * ca + CL_.q * sa;
    double CX_deltaE_a = -CD_.delta_e * ca + CL_.delta_e * sa;

    double CZ_a = -CD_a * sa - CL_a * ca;
    double CZ_q_a = -CD_.q * sa - CL_.q * ca;
    double CZ_deltaE_a = -CD_.delta_e * sa - CL_.delta_e * ca;

    forces(0) = 0.5 * (rho_)*Va * Va * wing_.S * (CX_a + (CX_q_a * wing_.c * q) / (2.0 * Va) + CX_deltaE_a * delta_.e)
                + 0.5 * rho_ * prop_.S * prop_.C * (pow((prop_.k_motor * delta_.t), 2.0) - Va * Va);
    forces(1) = 0.5 * (rho_)*Va * Va * wing_.S
                * (CY_.O + CY_.beta * beta + ((CY_.p * wing_.b * p) / (2.0 * Va)) + ((CY_.r * wing_.b * r) / (2.0 * Va))
                   + CY_.delta_a * delta_.a + CY_.delta_r * delta_.r);
    forces(2) = 0.5 * (rho_)*Va * Va * wing_.S * (CZ_a + (CZ_q_a * wing_.c * q) / (2.0 * Va) + CZ_deltaE_a * delta_.e);

    forces(3) = 0.5 * (rho_)*Va * Va * wing_.S * wing_.b
                    * (Cell_.O + Cell_.beta * beta + (Cell_.p * wing_.b * p) / (2.0 * Va)
                       + (Cell_.r * wing_.b * r) / (2.0 * Va) + Cell_.delta_a * delta_.a + Cell_.delta_r * delta_.r)
                - prop_.k_T_P * pow((prop_.k_Omega * delta_.t), 2.0);
    forces(4) = 0.5 * (rho_)*Va * Va * wing_.S * wing_.c
                * (Cm_.O + Cm_.alpha * alpha + (Cm_.q * wing_.c * q) / (2.0 * Va) + Cm_.delta_e * delta_.e);
    forces(5) = 0.5 * (rho_)*Va * Va * wing_.S * wing_.b
                * (Cn_.O + Cn_.beta * beta + (Cn_.p * wing_.b * p) / (2.0 * Va) + (Cn_.r * wing_.b * r) / (2.0 * Va)
                   + Cn_.delta_a * delta_.a + Cn_.delta_r * delta_.r);
  }
  else
  {
    forces(0) = 0.5 * rho_ * prop_.S * prop_.C * ((prop_.k_motor * delta_.t * prop_.k_motor * delta_.t));
    forces(1) = 0.0;
    forces(2) = 0.0;
    forces(3) = 0.0;
    forces(4) = 0.0;
    forces(5) = 0.0;
  }

  return forces;
}

void Fixedwing::set_wind(Eigen::Vector3d wind)
{
  wind_ = wind;
}

} // namespace rosflight_sim
