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

#include "fixedwing_forces_and_moments.hpp"

namespace rosflight_sim
{
Fixedwing::Fixedwing()
    : ForcesAndMomentsInterface()
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
  // Declare parameters
  declare_fixedwing_params();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&Fixedwing::parameters_callback, this, std::placeholders::_1));

  // Load all params from ROS. Throw an error if params don't exist
  update_params_from_ROS();
}

void Fixedwing::declare_fixedwing_params()
{
  this->declare_parameter("rho", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("mass", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("wing_s", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wing_b", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wing_c", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wing_M", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wing_epsilon", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("wing_alpha0", rclcpp::PARAMETER_DOUBLE);

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

  this->declare_parameter("D_prop", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("CT_0", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("CT_1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("CT_2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("CQ_0", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("CQ_1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("CQ_2", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("KV", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("KQ", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("V_max", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("R_motor", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("I_0", rclcpp::PARAMETER_DOUBLE);

  // RC command channel configuration
  this->declare_parameter("aileron_channel", 0);
  this->declare_parameter("elevator_channel", 1);
  this->declare_parameter("throttle_channel", 4);
  this->declare_parameter("rudder_channel", 2);

  // Servo time delay parameters
  this->declare_parameter("servo_refresh_rate", 0.003);
  this->declare_parameter("servo_tau", 0.01);
}

void Fixedwing::update_params_from_ROS()
{
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
  if (!this->get_parameter("D_prop", prop_.D_prop)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'D_prop' not defined");
  }
  if (!this->get_parameter("CT_0", prop_.CT_0)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CT_0' not defined");
  }
  if (!this->get_parameter("CT_1", prop_.CT_1)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CT_1' not defined");
  }
  if (!this->get_parameter("CT_2", prop_.CT_2)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CT_2' not defined");
  }
  if (!this->get_parameter("CQ_0", prop_.CQ_0)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CQ_0' not defined");
  }
  if (!this->get_parameter("CQ_1", prop_.CQ_1)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CQ_1' not defined");
  }
  if (!this->get_parameter("CQ_2", prop_.CQ_2)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'CQ_2' not defined");
  }

  // Motor Coefficients
  if (!this->get_parameter("KV", motor_.KV)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'KV' not defined");
  }
  if (!this->get_parameter("KQ", motor_.KQ)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'KQ' not defined");
  }
  if (!this->get_parameter("V_max", motor_.V_max)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'V_max' not defined");
  }
  if (!this->get_parameter("R_motor", motor_.R_motor)) {
    RCLCPP_ERROR(this->get_logger(), "Param 'R_motor' not defined");
  }
  if (!this->get_parameter("I_0", motor_.I_0)) {
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

rcl_interfaces::msg::SetParametersResult Fixedwing::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (auto param : parameters) {

    if (param.get_name() == "rho") {
			rho_ = param.as_double();
    }

    // Wing Geometry
    else if (param.get_name() == "wing_s") {
			wing_.S = param.as_double();
    }
    else if (param.get_name() == "wing_b") {
			wing_.b = param.as_double();
    }
    else if (param.get_name() == "wing_c") {
			wing_.c = param.as_double();
    }
    else if (param.get_name() == "wing_M") {
			wing_.M = param.as_double();
    }
    else if (param.get_name() == "wing_epsilon") {
			wing_.epsilon = param.as_double();
    }
    else if (param.get_name() == "wing_alpha0") {
			wing_.alpha0 = param.as_double();
    }

    // Propeller Coefficients
    else if (param.get_name() == "D_prop") {
			prop_.D_prop = param.as_double();
    }
    else if (param.get_name() == "CT_0") {
			prop_.CT_0 = param.as_double();
    }
    else if (param.get_name() == "CT_1") {
			prop_.CT_1 = param.as_double();
    }
    else if (param.get_name() == "CT_2") {
			prop_.CT_2 = param.as_double();
    }
    else if (param.get_name() == "CQ_0") {
			prop_.CQ_0 = param.as_double();
    }
    else if (param.get_name() == "CQ_1") {
			prop_.CQ_1 = param.as_double();
    }
    else if (param.get_name() == "CQ_2") {
			prop_.CQ_2 = param.as_double();
    }

    // Motor Coefficients
    else if (param.get_name() == "KV") {
			motor_.KV = param.as_double();
    }
    else if (param.get_name() == "KQ") {
			motor_.KQ = param.as_double();
    }
    else if (param.get_name() == "V_max") {
			motor_.V_max = param.as_double();
    }
    else if (param.get_name() == "R_motor") {
			motor_.R_motor = param.as_double();
    }
    else if (param.get_name() == "I_0") {
			motor_.I_0 = param.as_double();
    }

    else if (param.get_name() == "servo_tau") {
			servo_tau_ = param.as_double();
    }

    // Lift Params
    else if (param.get_name() == "C_L_O") {
			CL_.O = param.as_double();
    }
    else if (param.get_name() == "C_L_alpha") {
			CL_.alpha = param.as_double();
    }
    else if (param.get_name() == "C_L_beta") {
			CL_.beta = param.as_double();
    }
    else if (param.get_name() == "C_L_p") {
			CL_.p = param.as_double();
    }
    else if (param.get_name() == "C_L_q") {
			CL_.q = param.as_double();
    }
    else if (param.get_name() == "C_L_r") {
			CL_.r = param.as_double();
    }
    else if (param.get_name() == "C_L_delta_a") {
			CL_.delta_a = param.as_double();
    }
    else if (param.get_name() == "C_L_delta_e") {
			CL_.delta_e = param.as_double();
    }
    else if (param.get_name() == "C_L_delta_r") {
			CL_.delta_r = param.as_double();
    }

    // Drag Params
    else if (param.get_name() == "C_D_O") {
			CD_.O = param.as_double();
    }
    else if (param.get_name() == "C_D_alpha") {
			CD_.alpha = param.as_double();
    }
    else if (param.get_name() == "C_D_beta") {
			CD_.beta = param.as_double();
    }
    else if (param.get_name() == "C_D_p") {
			CD_.p = param.as_double();
    }
    else if (param.get_name() == "C_D_q") {
			CD_.q = param.as_double();
    }
    else if (param.get_name() == "C_D_r") {
			CD_.r = param.as_double();
    }
    else if (param.get_name() == "C_D_delta_a") {
			CD_.delta_a = param.as_double();
    }
    else if (param.get_name() == "C_D_delta_e") {
			CD_.delta_e = param.as_double();
    }
    else if (param.get_name() == "C_D_delta_r") {
			CD_.delta_r = param.as_double();
    }

    // ell Params (x axis moment)
    else if (param.get_name() == "C_ell_O") {
			Cell_.O = param.as_double();
    }
    else if (param.get_name() == "C_ell_alpha") {
			Cell_.alpha = param.as_double();
    }
    else if (param.get_name() == "C_ell_beta") {
			Cell_.beta = param.as_double();
    }
    else if (param.get_name() == "C_ell_p") {
			Cell_.p = param.as_double();
    }
    else if (param.get_name() == "C_ell_q") {
			Cell_.q = param.as_double();
    }
    else if (param.get_name() == "C_ell_r") {
			Cell_.r = param.as_double();
    }
    else if (param.get_name() == "C_ell_delta_a") {
			Cell_.delta_a = param.as_double();
    }
    else if (param.get_name() == "C_ell_delta_e") {
			Cell_.delta_e = param.as_double();
    }
    else if (param.get_name() == "C_ell_delta_r") {
			Cell_.delta_r = param.as_double();
    }

    // m Params (y axis moment)
    else if (param.get_name() == "C_m_O") {
			Cm_.O = param.as_double();
    }
    else if (param.get_name() == "C_m_alpha") {
			Cm_.alpha = param.as_double();
    }
    else if (param.get_name() == "C_m_beta") {
			Cm_.beta = param.as_double();
    }
    else if (param.get_name() == "C_m_p") {
			Cm_.p = param.as_double();
    }
    else if (param.get_name() == "C_m_q") {
			Cm_.q = param.as_double();
    }
    else if (param.get_name() == "C_m_r") {
			Cm_.r = param.as_double();
    }
    else if (param.get_name() == "C_m_delta_a") {
			Cm_.delta_a = param.as_double();
    }
    else if (param.get_name() == "C_m_delta_e") {
			Cm_.delta_e = param.as_double();
    }
    else if (param.get_name() == "C_m_delta_r") {
			Cm_.delta_r = param.as_double();
    }

    // n Params (z axis moment)
    else if (param.get_name() == "C_n_O") {
			Cn_.O = param.as_double();
    }
    else if (param.get_name() == "C_n_alpha") {
			Cn_.alpha = param.as_double();
    }
    else if (param.get_name() == "C_n_beta") {
			Cn_.beta = param.as_double();
    }
    else if (param.get_name() == "C_n_p") {
			Cn_.p = param.as_double();
    }
    else if (param.get_name() == "C_n_q") {
			Cn_.q = param.as_double();
    }
    else if (param.get_name() == "C_n_r") {
			Cn_.r = param.as_double();
    }
    else if (param.get_name() == "C_n_delta_a") {
			Cn_.delta_a = param.as_double();
    }
    else if (param.get_name() == "C_n_delta_e") {
			Cn_.delta_e = param.as_double();
    }
    else if (param.get_name() == "C_n_delta_r") {
			Cn_.delta_r = param.as_double();
    }

    // Y Params (Sideslip Forces)
    else if (param.get_name() == "C_Y_O") {
			CY_.O = param.as_double();
    }
    else if (param.get_name() == "C_Y_alpha") {
			CY_.alpha = param.as_double();
    }
    else if (param.get_name() == "C_Y_beta") {
			CY_.beta = param.as_double();
    }
    else if (param.get_name() == "C_Y_p") {
			CY_.p = param.as_double();
    }
    else if (param.get_name() == "C_Y_q") {
			CY_.q = param.as_double();
    }
    else if (param.get_name() == "C_Y_r") {
			CY_.r = param.as_double();
    }
    else if (param.get_name() == "C_Y_delta_a") {
			CY_.delta_a = param.as_double();
    }
    else if (param.get_name() == "C_Y_delta_e") {
			CY_.delta_e = param.as_double();
    }
    else if (param.get_name() == "C_Y_delta_r") {
			CY_.delta_r = param.as_double();
    }
  }

  return result;
}

geometry_msgs::msg::WrenchStamped Fixedwing::update_forces_and_torques(rosflight_msgs::msg::SimState x,
                                                                       geometry_msgs::msg::Vector3Stamped wind,
                                                                       std::array<uint16_t, 14> act_cmds)
{
  delta_.a = (act_cmds[this->get_parameter("aileron_channel").as_int()] - 1500.0) / 500.0;
  delta_.e = (act_cmds[this->get_parameter("elevator_channel").as_int()] - 1500.0) / 500.0;
  delta_.t = (act_cmds[this->get_parameter("throttle_channel").as_int()] - 1000.0) / 1000.0;
  delta_.r = (act_cmds[this->get_parameter("rudder_channel").as_int()] - 1500.0) / 500.0;

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
  geometry_msgs::msg::Vector3Stamped V_wind;  // in body frame
  geometry_msgs::msg::TransformStamped rot;
  rot.transform.rotation = x.pose.orientation;
  tf2::doTransform(wind, V_wind, rot);
  geometry_msgs::msg::Vector3Stamped V_airspeed;

  // Va = [ur, vr, wr] = V_ground - V_wind
  double ur = x.twist.linear.x - V_wind.vector.x;
  double vr = x.twist.linear.y - V_wind.vector.y;
  double wr = x.twist.linear.z - V_wind.vector.z;

  double Va = sqrt(pow(ur,2.0) + pow(vr,2.0) + pow(wr,2.0));

  geometry_msgs::msg::WrenchStamped forces;

  /*
   * The following math follows the method described in chapter 4 of
   * Small Unmanned Aircraft: Theory and Practice
   * By Randy Beard and Tim McLain.
   * Look there for a detailed explanation of each line in the rest of this function
   */

  double v_in = motor_.V_max * delta_.t;

  double a = (prop_.CQ_0 * (rho_) * (pow((prop_.D_prop), 5.0)) / (pow((2 * M_PI), 2.0)));
  double b = (prop_.CQ_1 * (rho_) * (pow((prop_.D_prop), 4.0)) / (2 * M_PI)) * Va
    + ((pow((motor_.KQ), 2.0)) / (motor_.R_motor));
  double c = (prop_.CQ_2 * (rho_) * (pow((prop_.D_prop), 3.0)) * (pow((Va), 2.0)))
    - ((motor_.KQ) / (motor_.R_motor)) * v_in + ((motor_.KQ) * (motor_.I_0));

  double Omega_p = ((-b + sqrt((pow((b), 2.0)) - (4 * a * c))) / (2 * a));

  double Prop_Force = ((rho_) * (pow((prop_.D_prop), 4.0))
                       * (((prop_.CT_0) * (pow((Omega_p), 2.0))) / (4 * (pow((M_PI), 2.0)))))
    + ((rho_) * (pow((prop_.D_prop), 3.0)) * (prop_.CT_1) * (Va) * (Omega_p) / (2 * M_PI))
    + ((rho_) * (pow((prop_.D_prop), 2.0)) * (prop_.CT_2) * (pow((Va), 2.0)));

  double Prop_Torque = ((rho_) * (pow((prop_.D_prop), 5.0))
                        * ((((prop_.CQ_0) / (4 * (pow((M_PI), 2.0))) * (pow((Omega_p), 2.0))))))
    + ((rho_) * (pow((prop_.D_prop), 4.0)) * (prop_.CQ_1) * (Va) * (Omega_p) / (2 * M_PI))
    + ((rho_) * (pow((prop_.D_prop), 3.0)) * (prop_.CQ_2) * (pow((Va), 2.0)));

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

    forces.wrench.force.x = 0.5 * (rho_) *Va * Va * wing_.S
        * (CX_a + (CX_q_a * wing_.c * q) / (2.0 * Va) + CX_deltaE_a * delta_curr.e)
      + Prop_Force;

    forces.wrench.force.y = 0.5 * (rho_) *Va * Va * wing_.S
      * (CY_.O + CY_.beta * beta + ((CY_.p * wing_.b * p) / (2.0 * Va))
         + ((CY_.r * wing_.b * r) / (2.0 * Va)) + CY_.delta_a * delta_curr.a
         + CY_.delta_r * delta_.r);

    forces.wrench.force.z = 0.5 * (rho_) *Va * Va * wing_.S
      * (CZ_a + (CZ_q_a * wing_.c * q) / (2.0 * Va) + CZ_deltaE_a * delta_curr.e);

    forces.wrench.torque.x = 0.5 * (rho_) *Va * Va * wing_.S * wing_.b
        * (Cell_.O + Cell_.beta * beta + (Cell_.p * wing_.b * p) / (2.0 * Va)
           + (Cell_.r * wing_.b * r) / (2.0 * Va) + Cell_.delta_a * delta_curr.a
           + Cell_.delta_r * delta_.r)
      - Prop_Torque;

    forces.wrench.torque.y = 0.5 * (rho_) *Va * Va * wing_.S * wing_.c
      * (Cm_.O + Cm_.alpha * alpha + (Cm_.q * wing_.c * q) / (2.0 * Va)
         + Cm_.delta_e * delta_curr.e);

    forces.wrench.torque.z = 0.5 * (rho_) *Va * Va * wing_.S * wing_.b
      * (Cn_.O + Cn_.beta * beta + (Cn_.p * wing_.b * p) / (2.0 * Va)
         + (Cn_.r * wing_.b * r) / (2.0 * Va) + Cn_.delta_a * delta_curr.a
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
}

} // namespace rosflight_sim


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosflight_sim::Fixedwing>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
