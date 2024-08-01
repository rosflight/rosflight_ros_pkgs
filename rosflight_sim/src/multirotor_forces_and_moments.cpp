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

#include <cmath>
#include <cstdint>
#include <iostream>
#include <rclcpp/parameter_value.hpp>
#include <rosflight_sim/multirotor_forces_and_moments.hpp>

namespace rosflight_sim
{
Multirotor::Multirotor(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
    , num_rotors_(0)
    , linear_mu_(0)
    , angular_mu_(0)
{
  declare_multirotor_params();


  num_rotors_ = node_->get_parameter("num_rotors").as_int();


  /* Load Rotor Configuration */
  motors_.resize(num_rotors_);
  
  Prop prop;

  prop.CT_0 = node_->get_parameter("CT_0").as_double();
  prop.CT_1 = node_->get_parameter("CT_1").as_double();
  prop.CT_2 = node_->get_parameter("CT_2").as_double();
  
  prop.CQ_0 = node_->get_parameter("CQ_0").as_double();
  prop.CQ_1 = node_->get_parameter("CQ_1").as_double();
  prop.CQ_2 = node_->get_parameter("CQ_2").as_double();

  prop.diam = node_->get_parameter("D_prop").as_double();
  
  std::vector<double> dists = node_->get_parameter("rotor_dists").as_double_array();
  std::vector<double> angles = node_->get_parameter("rotor_radial_angles").as_double_array();
  std::vector<int64_t> rotation_dirs = node_->get_parameter("rotor_rotation_directions").as_integer_array();
  
  Motor motor_characteristics;
  motor_characteristics.prop = prop;
  motor_characteristics.R = node_->get_parameter("R_motor").as_double();
  motor_characteristics.I_0 = node_->get_parameter("I_0").as_double();
  motor_characteristics.KQ = node_->get_parameter("KQ").as_double();
  

  for (int i = 0; i < num_rotors_; i++) {
    Motor motor = motor_characteristics;
    motor.dist = dists[i];
    motor.radial_angle = angles[i] * M_PI / 180.0;
    motor.direction = rotation_dirs[i];
    motor.command = 0.0;
    motors_[i] = motor;
  }

  wind_ = Eigen::Vector3d::Zero();

  forces_moments_pub_ =
    node_->create_publisher<geometry_msgs::msg::TwistStamped>("/forces_and_moments", 1);

}

Multirotor::~Multirotor() = default;

void Multirotor::declare_multirotor_params()
{
  // node_->declare_parameter("mass", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("num_rotors", rclcpp::PARAMETER_INTEGER);

  node_->declare_parameter("CT_0", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("CT_1", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("CT_2", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("CQ_0", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("CQ_1", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("CQ_2", rclcpp::PARAMETER_DOUBLE);
  
  node_->declare_parameter("KQ", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("V_max", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("R_motor", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("I_0", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("rotor_dists", rclcpp::PARAMETER_DOUBLE_ARRAY);
  node_->declare_parameter("rotor_radial_angles", rclcpp::PARAMETER_DOUBLE_ARRAY);
  node_->declare_parameter("rotor_rotation_directions", rclcpp::PARAMETER_INTEGER_ARRAY);

  node_->declare_parameter("D_prop", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("rho", rclcpp::PARAMETER_DOUBLE); // TODO: make sure these have default parameters.
}

Eigen::Matrix<double, 6, 1> Multirotor::update_forces_and_torques(CurrentState x,
                                                                  const int act_cmds[])
{
  // The maximum voltage of the battery.
  double V_max = node_->get_parameter("V_max").as_double();

  // double KQ = node_->get_parameter("KQ").as_double();
  // double R_motor = node_->get_parameter("R_motor").as_double();
  // double I_0 = node_->get_parameter("I_0").as_double();
  
  double rho = node_->get_parameter("rho").as_double();

  // Get airspeed vector for drag force calculation (rotate wind into body frame and add to inertial velocity)
  Eigen::Vector3d Va = x.vel + x.rot.inverse() * wind_;

  double total_thrust = 0.0;
  double total_roll_torque = 0.0;
  double total_pitch_torque = 0.0;
  double total_yaw_torque = 0.0;

  for (int i = 0; i < num_rotors_; i++) {

    double motor_cmd = (act_cmds[i] - 1000)/1000.0;

    motors_[i].command = motor_cmd;
  }

  for (Motor motor : motors_) {
    // Calculate prop_speed
    // Use model with V_max
    double v_in = V_max * motor.command;

    double a = (motor.prop.CQ_0 * (rho) * (pow((motor.prop.diam), 5.0)) / (pow((2 * M_PI), 2.0)));
    double b = (motor.prop.CQ_1 * (rho) * (pow((motor.prop.diam), 4.0)) / (2 * M_PI)) * Va.norm() // FIXME: The Va.norm() is actually just the portion parrallel to the prop's thrust.
      + ((pow((motor.KQ), 2.0)) / (motor.R));
    double c = (motor.prop.CQ_2 * (rho) * (pow((motor.prop.diam), 3.0)) * (pow(Va.norm(), 2.0)))
      - ((motor.KQ) / (motor.R)) * v_in + ((motor.KQ) * (motor.I_0));

    double prop_speed = ((-b + sqrt((pow((b), 2.0)) - (4 * a * c))) / (2 * a));

    // std::cout << "angle: " << motor.radial_angle << "\n";

    // Calculate the advance ratio.
    double advance_ratio = (2. * M_PI * Va.norm())/(prop_speed*motor.prop.diam);

    double CT = motor.prop.CT_2*pow(advance_ratio, 2) + motor.prop.CT_1*advance_ratio + motor.prop.CT_0;

    double motor_thrust = CT * (rho * pow(motor.prop.diam, 4))/(4*pow(M_PI,2)) * pow(prop_speed, 2);
    
    double motor_roll_torque = -motor_thrust*motor.dist*sinf(motor.radial_angle);
    double motor_pitch_torque = motor_thrust*motor.dist*cosf(motor.radial_angle);
    
    // std::cout << "motor_roll_torque " << motor_roll_torque << "\n";

    double CQ = motor.prop.CQ_2*pow(advance_ratio, 2) + motor.prop.CQ_1*advance_ratio + motor.prop.CQ_0;
    // std::cout << "CQ " << CQ << "\n";
    
    // The torque produced by the propeller spinning.
    double prop_torque = motor.direction * CQ * (rho*pow(motor.prop.diam, 5))/(4*pow(M_PI,2)) * pow(prop_speed, 2); // TODO: Do you need this?

    total_thrust += motor_thrust; // Okay
    total_roll_torque += motor_roll_torque;
    total_pitch_torque += motor_pitch_torque;
    total_yaw_torque += prop_torque; // Okay
  }

  // Rotate from body to inertial frame.
  
  Eigen::Vector3d body_forces;
  body_forces << 0.0, 0.0, -total_thrust;
  
  Eigen::Vector3d body_torques;
  body_torques << total_roll_torque, total_pitch_torque, -total_yaw_torque;

  Eigen::Matrix<double, 6, 1> forces;
  forces.block<3,1>(0,0) = body_forces;
  forces.block<3,1>(3,0) = body_torques;

  std::cout << forces << std::endl;

  // TODO: Apply drag.

  // TODO: Apply ground effect
  
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

void Multirotor::set_wind(Eigen::Vector3d wind) { wind_ = wind; }

} // namespace rosflight_sim
