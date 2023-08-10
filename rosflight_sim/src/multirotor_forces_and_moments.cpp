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

#include <rosflight_sim/multirotor_forces_and_moments.hpp>

namespace rosflight_sim
{
Multirotor::Multirotor(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)), num_rotors_(0), linear_mu_(0), angular_mu_(0)
{
  declareMultirotorParams();

  if (!node_->get_parameter("linear_mu", linear_mu_)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'linear_mu' not defined");
  }
  if (!node_->get_parameter("angular_mu", angular_mu_)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'angular_mu' not defined");
  }
  if (!node_->get_parameter("ground_effect", ground_effect_)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'ground_effect' not defined");
  }

  if (!node_->get_parameter("num_rotors", num_rotors_)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'num_rotors' not defined");
  }

  std::vector<double> rotor_positions(3 * num_rotors_);
  std::vector<double> rotor_vector_normal(3 * num_rotors_);
  std::vector<long> rotor_rotation_directions(num_rotors_);

  Rotor rotor;
  if (!node_->get_parameter("rotor_positions", rotor_positions)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'rotor_positions' not defined");
  }
  if (!node_->get_parameter("rotor_vector_normal", rotor_vector_normal)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'rotor_vector_normal' not defined");
  }

  if (!node_->get_parameter("rotor_rotation_directions", rotor_rotation_directions)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'rotor_rotation_directions' not defined");
  }
  if (!node_->get_parameter("rotor_max_thrust", rotor.max)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'rotor_max_thrust' not defined");
  }
  if (!node_->get_parameter("rotor_F", rotor.F_poly)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'rotor_F' not defined");
  }
  if (!node_->get_parameter("rotor_T", rotor.T_poly)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'rotor_T' not defined");
  }
  if (!node_->get_parameter("rotor_tau_up", rotor.tau_up)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'rotor_tau_up' not defined");
  }
  if (!node_->get_parameter("rotor_tau_down", rotor.tau_down)) {
    RCLCPP_ERROR(node_->get_logger(), "Param 'rotor_tau_down' not defined");
  }

  /* Load Rotor Configuration */
  motors_.resize(num_rotors_);

  force_allocation_matrix_.resize(4, num_rotors_);
  torque_allocation_matrix_.resize(4, num_rotors_);
  for (int i = 0; i < num_rotors_; i++) {
    motors_[i].rotor = rotor;
    motors_[i].position.resize(3);
    motors_[i].normal.resize(3);
    for (int j = 0; j < 3; j++) {
      motors_[i].position(j) = rotor_positions[3 * i + j];
      motors_[i].normal(j) = rotor_vector_normal[3 * i + j];
    }
    motors_[i].normal.normalize();
    motors_[i].direction = (int) rotor_rotation_directions[i];

    Eigen::Vector3d moment_from_thrust = motors_[i].position.cross(motors_[i].normal);
    Eigen::Vector3d moment_from_torque = motors_[i].direction * motors_[i].normal;

    // build allocation_matrices
    force_allocation_matrix_(0, i) = moment_from_thrust(0); // l
    force_allocation_matrix_(1, i) = moment_from_thrust(1); // m
    force_allocation_matrix_(2, i) = moment_from_thrust(2); // n
    force_allocation_matrix_(3, i) = motors_[i].normal(2);  // F

    torque_allocation_matrix_(0, i) = moment_from_torque(0); // l
    torque_allocation_matrix_(1, i) = moment_from_torque(1); // m
    torque_allocation_matrix_(2, i) = moment_from_torque(2); // n
    torque_allocation_matrix_(3, i) = 0.0;                   // F
  }

  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "allocation matrices:\nFORCE \n"
                       << force_allocation_matrix_ << "\nTORQUE\n"
                       << torque_allocation_matrix_ << "\n");

  // Initialize size of dynamic force and torque matrices
  desired_forces_.resize(num_rotors_);
  desired_torques_.resize(num_rotors_);
  actual_forces_.resize(num_rotors_);
  actual_torques_.resize(num_rotors_);

  for (int i = 0; i < num_rotors_; i++) {
    desired_forces_(i) = 0.0;
    desired_torques_(i) = 0.0;
    actual_forces_(i) = 0.0;
    actual_torques_(i) = 0.0;
  }

  wind_ = Eigen::Vector3d::Zero();
  prev_time_ = -1;
}

Multirotor::~Multirotor() = default;

void Multirotor::declareMultirotorParams()
{
  node_->declare_parameter("mass", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("linear_mu", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("angular_mu", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("ground_effect", rclcpp::PARAMETER_DOUBLE_ARRAY);

  node_->declare_parameter("num_rotors", rclcpp::PARAMETER_INTEGER);
  node_->declare_parameter("rotor_positions", rclcpp::PARAMETER_DOUBLE_ARRAY);
  node_->declare_parameter("rotor_vector_normal", rclcpp::PARAMETER_DOUBLE_ARRAY);

  node_->declare_parameter("rotor_rotation_directions", rclcpp::PARAMETER_INTEGER_ARRAY);
  node_->declare_parameter("rotor_max_thrust", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("rotor_F", rclcpp::PARAMETER_DOUBLE_ARRAY);
  node_->declare_parameter("rotor_T", rclcpp::PARAMETER_DOUBLE_ARRAY);
  node_->declare_parameter("rotor_tau_up", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("rotor_tau_down", rclcpp::PARAMETER_DOUBLE);
}

Eigen::Matrix<double, 6, 1> Multirotor::updateForcesAndTorques(Current_State x,
                                                               const int act_cmds[])
{
  if (prev_time_ < 0) {
    prev_time_ = x.t;
    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  double dt = x.t - prev_time_;
  double pd = x.pos[2];

  // Get airspeed vector for drag force calculation (rotate wind into body frame and add to inertial velocity)
  Eigen::Vector3d Va = x.vel + x.rot.inverse() * wind_;

  // Calculate Forces
  for (int i = 0; i < num_rotors_; i++) {
    // First, figure out the desired force output from passing the signal into the quadratic approximation
    double signal = act_cmds[i];
    desired_forces_(i, 0) = motors_[i].rotor.F_poly[0] * signal * signal
      + motors_[i].rotor.F_poly[1] * signal + motors_[i].rotor.F_poly[2];
    desired_torques_(i, 0) = motors_[i].rotor.T_poly[0] * signal * signal
      + motors_[i].rotor.T_poly[1] * signal + motors_[i].rotor.T_poly[2];

    // Then, Calculate Actual force and torque for each rotor using first-order dynamics
    double tau = (desired_forces_(i, 0) > actual_forces_(i, 0)) ? motors_[i].rotor.tau_up
                                                                : motors_[i].rotor.tau_down;
    double alpha = dt / (tau + dt);
    actual_forces_(i, 0) =
      sat((1 - alpha) * actual_forces_(i) + alpha * desired_forces_(i), motors_[i].rotor.max, 0.0);
    actual_torques_(i, 0) = sat((1 - alpha) * actual_torques_(i) + alpha * desired_torques_(i),
                                motors_[i].rotor.max, 0.0);
  }

  // Use the allocation matrix to calculate the body-fixed force and torques
  Eigen::Vector4d output_forces = force_allocation_matrix_ * actual_forces_;
  Eigen::Vector4d output_torques = torque_allocation_matrix_ * actual_torques_;
  Eigen::Vector4d output_forces_and_torques = output_forces + output_torques;

  // Calculate Ground Effect
  double z = -pd;
  double ground_effect =
    max(ground_effect_[0] * z * z * z * z + ground_effect_[1] * z * z * z
          + ground_effect_[2] * z * z + ground_effect_[3] * z + ground_effect_[4],
        0);

  Eigen::Matrix<double, 6, 1> forces;
  // Apply other forces (drag) <- follows "Quadrotors and Accelerometers - State Estimation With an Improved Dynamic
  // Model" By Rob Leishman et al.
  forces.block<3, 1>(0, 0) = -linear_mu_ * Va;
  forces.block<3, 1>(3, 0) = -angular_mu_ * x.omega + output_forces_and_torques.block<3, 1>(0, 0);

  // Apply ground effect and thrust
  forces(2) += output_forces_and_torques(3) - ground_effect;

  return forces;
}

void Multirotor::set_wind(Eigen::Vector3d wind) { wind_ = wind; }

} // namespace rosflight_sim
