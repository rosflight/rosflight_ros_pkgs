/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2025 Jacob Moore
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

#include "standalone_dynamics.hpp"

namespace rosflight_sim
{

StandaloneDynamics::StandaloneDynamics()
  : DynamicsInterface()
{
  current_truth_state_ = rosflight_msgs::msg::SimState();

  // Declare parameters and set up the callback for changing parameters
  declare_parameters();
  // TODO: parameter callback...

  compute_inertia_matrix();
}

void StandaloneDynamics::declare_parameters()
{
  this->declare_parameter("mass", 4.5);
  this->declare_parameter("Jxx", 0.24855);
  this->declare_parameter("Jxy", 0.0);
  this->declare_parameter("Jxz", 0.06);
  this->declare_parameter("Jyy", 0.3784);
  this->declare_parameter("Jyz", 0.0);
  this->declare_parameter("Jzz", 0.618);
  this->declare_parameter("gravity", 9.81);
}

void StandaloneDynamics::compute_inertia_matrix()
{
  J_ << this->get_parameter("Jxx").as_double(), 
    this->get_parameter("Jxy").as_double(), 
    this->get_parameter("Jxz").as_double(), 
    this->get_parameter("Jxy").as_double(), 
    this->get_parameter("Jyy").as_double(), 
    this->get_parameter("Jyz").as_double(), 
    this->get_parameter("Jxz").as_double(), 
    this->get_parameter("Jyz").as_double(), 
    this->get_parameter("Jzz").as_double();

  J_inv_ = J_.inverse();
}

void StandaloneDynamics::apply_forces_and_torques(const geometry_msgs::msg::WrenchStamped & forces_torques)
{
  double dt = compute_dt(forces_torques.header.stamp.sec + forces_torques.header.stamp.nanosec * 1e-9);
  if (dt == 0.0) { return; } // Don't integrate if the timestep is zero

  // Forces should aleady be in the body frame
  Eigen::VectorXd fm(6);
  fm << forces_torques.wrench.force.x,
        forces_torques.wrench.force.y,
        forces_torques.wrench.force.z,
        forces_torques.wrench.torque.x,
        forces_torques.wrench.torque.y,
        forces_torques.wrench.torque.z;

  // Add gravity to the aerodynamics and propulsion forces
  fm = add_gravity_forces(fm);

  // Add ground collision forces - low fidelity approximation
  fm = add_ground_collision_forces(fm);

  // Fill in state Eigen vectors
  Eigen::VectorXd state(13);
  state << current_truth_state_.pose.position.x,
           current_truth_state_.pose.position.y,
           current_truth_state_.pose.position.z,
           current_truth_state_.twist.linear.x,
           current_truth_state_.twist.linear.y,
           current_truth_state_.twist.linear.z,
           current_truth_state_.pose.orientation.w,
           current_truth_state_.pose.orientation.x,
           current_truth_state_.pose.orientation.y,
           current_truth_state_.pose.orientation.z,
           current_truth_state_.twist.angular.x,
           current_truth_state_.twist.angular.y,
           current_truth_state_.twist.angular.z;

  // integrate forces and torques
  rk4(state, fm, dt);
}

Eigen::VectorXd StandaloneDynamics::add_gravity_forces(Eigen::VectorXd forces)
{
  // Rotate gravity into body frame
  Eigen::Quaterniond q_body_to_inertial(current_truth_state_.pose.orientation.w, 
                                        current_truth_state_.pose.orientation.x,
                                        current_truth_state_.pose.orientation.y,
                                        current_truth_state_.pose.orientation.z);
  Eigen::Vector3d gravity(0,0,this->get_parameter("gravity").as_double());
  Eigen::Vector3d gravity_body = q_body_to_inertial.inverse() * gravity;

  forces.segment(0,3) += gravity_body * this->get_parameter("mass").as_double();
  return forces;
}

Eigen::VectorXd StandaloneDynamics::add_ground_collision_forces(Eigen::VectorXd forces)
{
  // Low fidelity approximation of what actually might happen
  // TODO: Current hypothesis: I am setting the forces equal to zero, but I am not setting the cross terms in the accelerations equal to zero.
  // Thus, the f/m portion of accel is zero, but the omega cross v term may not be zero.
  if (current_truth_state_.pose.position.z > -0.01) { // NED frame
    // If close to the ground
    current_truth_state_.pose.position.z = std::min(0.0, current_truth_state_.pose.position.z);

    Eigen::Quaterniond q_body_to_inertial{current_truth_state_.pose.orientation.w, 
                                          current_truth_state_.pose.orientation.x,
                                          current_truth_state_.pose.orientation.y,
                                          current_truth_state_.pose.orientation.z};
    Eigen::Vector3d forces_in_body_frame = forces.segment(0,3);
    Eigen::Vector3d moments_in_body_frame = forces.segment(3,3);
    Eigen::Vector3d forces_in_inertial_frame = q_body_to_inertial * forces_in_body_frame;
    Eigen::Vector3d moments_in_inertial_frame = q_body_to_inertial * moments_in_body_frame;

    if (forces_in_inertial_frame(2) > 0.0) {
      // If down force is positive, set it to zero.
      forces_in_inertial_frame(2) = 0.0;
      forces.segment(0,3) = q_body_to_inertial.inverse() * forces_in_inertial_frame;
    }

    // Also set roll moments to zero if close to the ground
    moments_in_inertial_frame(0) = 0.0;
    moments_in_inertial_frame(2) = 0.0;
    forces.segment(3,3) = q_body_to_inertial.inverse() * moments_in_inertial_frame;
    // tf2::Quaternion q(current_truth_state_.pose.orientation.x,
    //                   current_truth_state_.pose.orientation.y,
    //                   current_truth_state_.pose.orientation.z,
    //                   current_truth_state_.pose.orientation.w);
    // tf2::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    //
    // q.setRPY(0.0,pitch,yaw);
    // current_truth_state_.pose.orientation = tf2::toMsg(q);
  }

  return forces;
}

void StandaloneDynamics::rk4(Eigen::VectorXd state, Eigen::VectorXd forces_moments, double dt)
{
  // RK4
  Eigen::VectorXd k1 = f(state, forces_moments);
  Eigen::VectorXd k2 = f(state + dt/2 * k1, forces_moments);
  Eigen::VectorXd k3 = f(state + dt/2 * k2, forces_moments);
  Eigen::VectorXd k4 = f(state + dt * k3, forces_moments);
  state += dt / 6 * (k1 + 2*k2 + 2*k3 + k4); // Integrate states

  // Normalize the quaternion
  state.segment(6,4).normalize();

  // Unpackage eigen vector into current state
  current_truth_state_.pose.position.x = state(0);
  current_truth_state_.pose.position.y = state(1);
  current_truth_state_.pose.position.z = state(2);
  current_truth_state_.twist.linear.x = state(3);
  current_truth_state_.twist.linear.y = state(4);
  current_truth_state_.twist.linear.z = state(5);
  current_truth_state_.pose.orientation.w = state(6);
  current_truth_state_.pose.orientation.x = state(7);
  current_truth_state_.pose.orientation.y = state(8);
  current_truth_state_.pose.orientation.z = state(9);
  current_truth_state_.twist.angular.x = state(10);
  current_truth_state_.twist.angular.y = state(11);
  current_truth_state_.twist.angular.z = state(12);

  Eigen::VectorXd accels = compute_accels_with_updated_state(state, forces_moments);
  current_truth_state_.acceleration.linear.x = accels(0);
  current_truth_state_.acceleration.linear.y = accels(1);
  current_truth_state_.acceleration.linear.z = accels(2);
  current_truth_state_.acceleration.angular.x = accels(3);
  current_truth_state_.acceleration.angular.y = accels(4);
  current_truth_state_.acceleration.angular.z = accels(5);
}

Eigen::VectorXd StandaloneDynamics::compute_accels_with_updated_state(Eigen::VectorXd state, Eigen::VectorXd forces_moments)
{
  Eigen::Vector3d force = forces_moments.head(3);
  Eigen::Vector3d torques = forces_moments.tail(3);
  double mass = this->get_parameter("mass").as_double();

  Eigen::Vector3d lin_accel = force / mass;
  Eigen::Vector3d angular_accel = J_inv_ * torques;

  Eigen::VectorXd accels(6);
  accels << lin_accel, angular_accel;
  return accels;
}

double StandaloneDynamics::compute_dt(double now)
{
  static double prev_time = 0;
  if(prev_time == 0) {
    prev_time = now;
    return 0;
  }

  // Calculate time
  double dt = now - prev_time;
  prev_time = now;

  return dt;
}

Eigen::VectorXd StandaloneDynamics::f(Eigen::VectorXd state, Eigen::VectorXd forces_moments)
{
  // State:
  // pn pe pd   - Inertial positions in inertial frame
  // u v w      - Inertial velocities in body frame
  // quaternion - Rotation from body to inertial
  // p q r      - Angular velocity about center of mass in body frame

  // Extract needed terms
  Eigen::Vector3d uvw = state.segment(3,3);
  Eigen::VectorXd quat = state.segment(6,4);
  Eigen::Vector3d pqr = state.segment(10,3);
  Eigen::Vector3d force = forces_moments.head(3);
  Eigen::Vector3d torques = forces_moments.tail(3);
  double mass = this->get_parameter("mass").as_double();

  // Compute x_dot = f(x, u)
  // Taken from Small Unmanned Aircraft: Theory and Practice by Beard, McLain
  Eigen::VectorXd out(13);

  // pn pe pd
  Eigen::Quaterniond q_body_to_inertial(quat(0), quat(1), quat(2), quat(3));
  Eigen::Vector3d pos_dot = q_body_to_inertial * uvw;
  out.segment(0,3) = pos_dot;

  // uvw -- force is in body frame
  Eigen::Vector3d uvw_dot = force / mass - pqr.cross(uvw);
  out.segment(3,3) = uvw_dot;

  // quat
  Eigen::Matrix4d omega_skew;
  omega_skew << 0, -pqr[0], -pqr[1], -pqr[2],
    pqr[0], 0, pqr[2], -pqr[1],
    pqr[1], -pqr[2], 0, pqr[0],
    pqr[2], pqr[1], -pqr[0], 0;
  Eigen::VectorXd quat_dot = 0.5 * omega_skew * quat;
  out.segment(6,4) = quat_dot;

  // pqr
  Eigen::Vector3d pqr_dot = J_inv_ * (-pqr.cross(J_ * pqr) + torques);
  out.segment(10,3) = pqr_dot;

  return out;
}

bool StandaloneDynamics::set_sim_state(const rosflight_msgs::msg::SimState state)
{
  current_truth_state_ = state;
  return true;
}

rosflight_msgs::msg::SimState StandaloneDynamics::compute_truth()
{
  return current_truth_state_;
}

geometry_msgs::msg::Vector3Stamped StandaloneDynamics::compute_wind_truth()
{
  // Wind in the inertial frame
  current_wind_truth_.header.stamp = this->now();
  // TODO: Add wind
  return current_wind_truth_;
}

} // namespace rosflight_sim


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosflight_sim::StandaloneDynamics>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
