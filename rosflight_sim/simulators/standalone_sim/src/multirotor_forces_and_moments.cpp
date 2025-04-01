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

#include "multirotor_forces_and_moments.hpp"

namespace rosflight_sim
{
Multirotor::Multirotor()
    : ForcesAndMomentsInterface()
    , num_rotors_(0)
{
  declare_multirotor_params();

  num_rotors_ = this->get_parameter("num_rotors").as_int();

  /* Load Rotor Configuration */
  motors_.resize(num_rotors_);

  Prop prop;

  prop.CT_0 = this->get_parameter("CT_0").as_double();
  prop.CT_1 = this->get_parameter("CT_1").as_double();
  prop.CT_2 = this->get_parameter("CT_2").as_double();

  prop.CQ_0 = this->get_parameter("CQ_0").as_double();
  prop.CQ_1 = this->get_parameter("CQ_1").as_double();
  prop.CQ_2 = this->get_parameter("CQ_2").as_double();

  prop.diam = this->get_parameter("D_prop").as_double();

  std::vector<double> dists = this->get_parameter("rotor_dists").as_double_array();
  std::vector<double> angles = this->get_parameter("rotor_radial_angles").as_double_array();
  std::vector<int64_t> rotation_dirs = this->get_parameter("rotor_rotation_directions").as_integer_array();

  Motor motor_characteristics;
  motor_characteristics.prop = prop;
  motor_characteristics.R = this->get_parameter("R_motor").as_double();
  motor_characteristics.I_0 = this->get_parameter("I_0").as_double();
  motor_characteristics.KQ = this->get_parameter("KQ").as_double();


  for (int i = 0; i < num_rotors_; i++) {
    Motor motor = motor_characteristics;
    motor.dist = dists[i];
    motor.radial_angle = angles[i] * M_PI / 180.0;
    motor.direction = rotation_dirs[i];
    motor.command = 0.0;
    motors_[i] = motor;
  }
}

Multirotor::~Multirotor() = default;

void Multirotor::declare_multirotor_params()
{
  this->declare_parameter("num_rotors", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("CT_0", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("CT_1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("CT_2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("CQ_0", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("CQ_1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("CQ_2", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("KQ", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("V_max", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("R_motor", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("I_0", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("rotor_dists", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("rotor_radial_angles", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("rotor_rotation_directions", rclcpp::PARAMETER_INTEGER_ARRAY);

  this->declare_parameter("D_prop", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("rho", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("CD", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("A_c", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("CD_induced", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("ground_effect", rclcpp::PARAMETER_DOUBLE_ARRAY);
}

geometry_msgs::msg::WrenchStamped Multirotor::update_forces_and_torques(rosflight_msgs::msg::SimState x,
                                                                        geometry_msgs::msg::Vector3Stamped wind,
                                                                        std::array<uint16_t, 14> act_cmds)
{
  // The maximum voltage of the battery.
  double V_max = this->get_parameter("V_max").as_double();

  // TODO: We should change this to the nonlinear model as a function of altitude
  double rho = this->get_parameter("rho").as_double();

  // Get airspeed vector for drag force calculation (rotate wind into
  // body frame and subtract from inertial velocity in body frame)
  geometry_msgs::msg::Vector3Stamped V_wind;  // in body frame
  geometry_msgs::msg::TransformStamped rot;
  rot.transform.rotation = x.pose.orientation;
  tf2::doTransform(wind, V_wind, rot);
  geometry_msgs::msg::Vector3Stamped V_airspeed;

  // Compute airspeed
  // Va = [ur, vr, wr] = V_ground - V_wind
  Eigen::Vector3d Vg(x.twist.linear.x, x.twist.linear.y, x.twist.linear.z);
  double ur = Vg(0) - V_wind.vector.x;
  double vr = Vg(1) - V_wind.vector.y;
  double wr = Vg(2) - V_wind.vector.z;
  Eigen::Vector3d Va(ur, vr, wr);

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

  for (Motor motor : motors_) {
    // Calculate prop_speed
    // Use model with V_max
    double v_in = V_max * motor.command;

    // Take true angle and only compare the portion parallel to the body z-axis.
    // TODO: Add better model of propeller with non-straight aero forces
    Eigen::Vector3d body_z;
    body_z << 0.0, 0.0, 1.0;

    double Va_along_z = (-Va).dot(body_z);

    // Saturate the airspeed to zero. The model therefore assumes that air flowing backwards
    // through the propeller has the same effect (on CT and CQ) as stationary air.
    if (Va_along_z < 0.0) {
      Va_along_z = 0.0;
    }

    double a = (motor.prop.CQ_0 * (rho) * (pow((motor.prop.diam), 5.0)) / (pow((2 * M_PI), 2.0)));
    double b = (motor.prop.CQ_1 * (rho) * (pow((motor.prop.diam), 4.0)) / (2 * M_PI)) * Va_along_z
      + ((pow((motor.KQ), 2.0)) / (motor.R));
    double c = (motor.prop.CQ_2 * (rho) * (pow((motor.prop.diam), 3.0)) * (pow(Va_along_z, 2.0)))
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

    double motor_thrust = CT * (rho * pow(motor.prop.diam, 4))/(4*pow(M_PI,2)) * pow(prop_speed, 2);

    double motor_roll_torque = -motor_thrust*motor.dist*sinf(motor.radial_angle);
    double motor_pitch_torque = motor_thrust*motor.dist*cosf(motor.radial_angle);

    double CQ = motor.prop.CQ_2*pow(advance_ratio, 2) + motor.prop.CQ_1*advance_ratio + motor.prop.CQ_0;
    if (CQ <= 0.0) {
      CQ = 0.0001;
    }

    // The torque produced by the propeller spinning.
    double prop_torque = motor.direction * CQ * (rho*pow(motor.prop.diam, 5))/(4*pow(M_PI,2)) * pow(prop_speed, 2);

    total_thrust += motor_thrust;
    total_roll_torque += motor_roll_torque;
    total_pitch_torque += motor_pitch_torque;
    total_yaw_torque += prop_torque;
  }

  // Compute aerodynamic drag forces on the body of the vehicle
  double cross_sectional_area = this->get_parameter("A_c").as_double(); // m^2
  double CD = this->get_parameter("CD").as_double();

  double drag = 0.5 * rho * cross_sectional_area * CD * pow(Va.norm(), 2);
  Eigen::Vector3d drag_force;

  if (Va.norm() < 0.001) {
    drag_force << 0.0, 0.0, 0.0;
  } else {
    drag_force = -drag * Va / Va.norm();
  }

  // Compute the drag force induced from the propellers
  double CD_induced = this->get_parameter("CD_induced").as_double();

  Eigen::Matrix3d drag_matrix;
  drag_matrix << -total_thrust*CD_induced, 0.0, 0.0,
                 0.0, -total_thrust*CD_induced, 0.0,
                 0.0, 0.0, 0.0;

  Eigen::Vector3d induced_drag_force = drag_matrix * Va;

  // Apply Ground Effect
  std::vector<double> ground_effect_coeffs = this->get_parameter("ground_effect").as_double_array();
  double ground_effect_force = max(0.0, ground_effect_coeffs[0]*pow(-x.pose.position.z, 4) +
                                        ground_effect_coeffs[1]*pow(-x.pose.position.z, 3) +
                                        ground_effect_coeffs[2]*pow(-x.pose.position.z, 2) +
                                        ground_effect_coeffs[3]*-x.pose.position.z +
                                        ground_effect_coeffs[4]);
  Eigen::Vector3d ground_effect(0.0, 0.0, -ground_effect_force);


  // Add the forces together in the body frame
  Eigen::Vector3d body_forces;
  body_forces << 0.0, 0.0, -total_thrust; // negative Z direction
  body_forces += drag_force;
  body_forces += induced_drag_force;
  body_forces += ground_effect;

  Eigen::Vector3d body_torques;
  body_torques << total_roll_torque, total_pitch_torque, -total_yaw_torque;

  // Package up message and return
  geometry_msgs::msg::WrenchStamped msg;

  msg.header.stamp = this->get_clock()->now();

  msg.wrench.force.x = body_forces(0);
  msg.wrench.force.y = body_forces(1);
  msg.wrench.force.z = body_forces(2);
  msg.wrench.torque.x = body_torques(0);
  msg.wrench.torque.y = body_torques(1);
  msg.wrench.torque.z = body_torques(2);

  return msg;
}

} // namespace rosflight_sim


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosflight_sim::Multirotor>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
