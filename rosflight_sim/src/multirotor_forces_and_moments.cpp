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

#include <rosflight_sim/multirotor_forces_and_moments.h>

namespace rosflight_sim
{

Multirotor::Multirotor(ros::NodeHandle *nh)
{
  nh_ = nh;

  // Pull Parameters off of rosparam server
  num_rotors_ = 0;
  ROS_ASSERT_MSG(nh_->getParam("ground_effect", ground_effect_), "missing parameters in %s namespace", nh_->getNamespace().c_str());
  ROS_ASSERT(nh_->getParam("mass", mass_));
  ROS_ASSERT(nh_->getParam("linear_mu", linear_mu_));
  ROS_ASSERT(nh_->getParam("angular_mu", angular_mu_));
  ROS_ASSERT(nh_->getParam("num_rotors", num_rotors_));

  std::vector<double> rotor_positions(3 * num_rotors_);
  std::vector<double> rotor_vector_normal(3 * num_rotors_);
  std::vector<int> rotor_rotation_directions(num_rotors_);

  // For now, just assume all rotors are the same
  Rotor rotor;

  ROS_ASSERT(nh_->getParam("rotor_positions", rotor_positions));
  ROS_ASSERT(nh_->getParam("rotor_vector_normal", rotor_vector_normal));
  ROS_ASSERT(nh_->getParam("rotor_rotation_directions", rotor_rotation_directions));
  ROS_ASSERT(nh_->getParam("rotor_max_thrust", rotor.max));
  ROS_ASSERT(nh_->getParam("rotor_F", rotor.F_poly));
  ROS_ASSERT(nh_->getParam("rotor_T", rotor.T_poly));
  ROS_ASSERT(nh_->getParam("rotor_tau_up", rotor.tau_up));
  ROS_ASSERT(nh_->getParam("rotor_tau_down", rotor.tau_down));

  /* Load Rotor Configuration */
  motors_.resize(num_rotors_);

  force_allocation_matrix_.resize(4,num_rotors_);
  torque_allocation_matrix_.resize(4,num_rotors_);
  for(int i = 0; i < num_rotors_; i++)
  {
    motors_[i].rotor = rotor;
    motors_[i].position.resize(3);
    motors_[i].normal.resize(3);
    for (int j = 0; j < 3; j++)
    {
      motors_[i].position(j) = rotor_positions[3*i + j];
      motors_[i].normal(j) = rotor_vector_normal[3*i + j];
    }
    motors_[i].normal.normalize();
    motors_[i].direction = rotor_rotation_directions[i];

    Eigen::Vector3d moment_from_thrust = motors_[i].position.cross(motors_[i].normal);
    Eigen::Vector3d moment_from_torque = motors_[i].direction * motors_[i].normal;

    // build allocation_matrices
    force_allocation_matrix_(0,i) = moment_from_thrust(0); // l
    force_allocation_matrix_(1,i) = moment_from_thrust(1); // m
    force_allocation_matrix_(2,i) = moment_from_thrust(2); // n
    force_allocation_matrix_(3,i) = motors_[i].normal(2); // F

    torque_allocation_matrix_(0,i) = moment_from_torque(0); // l
    torque_allocation_matrix_(1,i) = moment_from_torque(1); // m
    torque_allocation_matrix_(2,i) = moment_from_torque(2); // n
    torque_allocation_matrix_(3,i) = 0.0; // F
  }

  ROS_INFO_STREAM("allocation matrices:\nFORCE \n" << force_allocation_matrix_ << "\nTORQUE\n" << torque_allocation_matrix_ << "\n");

  // Initialize size of dynamic force and torque matrices
  desired_forces_.resize(num_rotors_);
  desired_torques_.resize(num_rotors_);
  actual_forces_.resize(num_rotors_);
  actual_torques_.resize(num_rotors_);

  for (int i = 0; i < num_rotors_; i++)
  {
    desired_forces_(i)=0.0;
    desired_torques_(i)=0.0;
    actual_forces_(i)=0.0;
    actual_torques_(i)=0.0;
  }

  wind_ = Eigen::Vector3d::Zero();
  prev_time_ = -1;
}

Eigen::Matrix<double, 6, 1> Multirotor::updateForcesAndTorques(Current_State x, const int act_cmds[])
{
  if (prev_time_ < 0)
  {
    prev_time_ = x.t;
    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  double dt = x.t - prev_time_;
  double pd = x.pos[2];

  // Get airspeed vector for drag force calculation (rotate wind into body frame and add to inertial velocity)
  Eigen::Vector3d Va = x.vel + x.rot.inverse()*wind_;

  // Calculate Forces
  for (int i = 0; i<num_rotors_; i++)
  {
    // First, figure out the desired force output from passing the signal into the quadratic approximation
    double signal = act_cmds[i];
    desired_forces_(i,0) = motors_[i].rotor.F_poly[0]*signal*signal + motors_[i].rotor.F_poly[1]*signal + motors_[i].rotor.F_poly[2];
    desired_torques_(i,0) = motors_[i].rotor.T_poly[0]*signal*signal + motors_[i].rotor.T_poly[1]*signal + motors_[i].rotor.T_poly[2];

    // Then, Calculate Actual force and torque for each rotor using first-order dynamics
    double tau = (desired_forces_(i,0) > actual_forces_(i,0)) ? motors_[i].rotor.tau_up : motors_[i].rotor.tau_down;
    double alpha = dt/(tau + dt);
    actual_forces_(i,0) = sat((1-alpha)*actual_forces_(i) + alpha*desired_forces_(i), motors_[i].rotor.max, 0.0);
    actual_torques_(i,0) = sat((1-alpha)*actual_torques_(i) + alpha*desired_torques_(i), motors_[i].rotor.max, 0.0);
  }

  // Use the allocation matrix to calculate the body-fixed force and torques
  Eigen::Vector4d output_forces = force_allocation_matrix_*actual_forces_;
  Eigen::Vector4d output_torques = torque_allocation_matrix_*actual_torques_;
  Eigen::Vector4d output_forces_and_torques = output_forces + output_torques;

  // Calculate Ground Effect
  double z = -pd;
  double ground_effect = max(ground_effect_[0]*z*z*z*z + ground_effect_[1]*z*z*z + ground_effect_[2]*z*z + ground_effect_[3]*z + ground_effect_[4], 0);

  Eigen::Matrix<double, 6,1> forces;
  // Apply other forces (drag) <- follows "Quadrotors and Accelerometers - State Estimation With an Improved Dynamic Model"
  // By Rob Leishman et al.
  forces.block<3,1>(3,0) = -linear_mu_ * Va;
  forces.block<3,1>(3,0) = -angular_mu_ * x.omega + output_forces_and_torques.block<3,1>(0,0);

  // Apply ground effect and thrust
  forces(2) += output_forces_and_torques(3) - ground_effect;

  return forces;
}

void Multirotor::set_wind(Eigen::Vector3d wind)
{
  wind_ = wind;
}

}
