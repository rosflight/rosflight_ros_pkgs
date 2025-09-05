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

#include "gazebo_dynamics.hpp"
#include "chrono"

using namespace std::chrono_literals;

namespace rosflight_sim
{

GazeboDynamics::GazeboDynamics(gazebo::physics::LinkPtr link, std::string link_name) : DynamicsInterface()
{
  link_ = link;
  link_name_ = link_name;
}

void GazeboDynamics::apply_forces_and_torques(const geometry_msgs::msg::WrenchStamped & forces_torques)
{
  // Gazebo expects the forces to be added in NWU
  GazeboVector force(forces_torques.wrench.force.x, -forces_torques.wrench.force.y, -forces_torques.wrench.force.z);
  GazeboVector torque(forces_torques.wrench.torque.x, -forces_torques.wrench.torque.y, -forces_torques.wrench.torque.z);

  link_->AddRelativeForce(force);
  link_->AddRelativeTorque(torque);
}

rosflight_msgs::msg::SimState GazeboDynamics::compute_truth()
{
  GazeboPose pose = GZ_COMPAT_GET_WORLD_COG_POSE(link_);
  GazeboVector vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  GazeboVector omega = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);
  GazeboVector lin_accel = GZ_COMPAT_GET_RELATIVE_LINEAR_ACCEL(link_);
  GazeboVector ang_accel = GZ_COMPAT_GET_RELATIVE_ANGULAR_ACCEL(link_);

  // Publish truth
  rosflight_msgs::msg::SimState truth;
  truth.header.stamp = this->now();
  truth.header.frame_id = link_name_ + "_NED"; 
  truth.pose.orientation.w = GZ_COMPAT_GET_W(GZ_COMPAT_GET_ROT(pose));
  truth.pose.orientation.x = GZ_COMPAT_GET_X(GZ_COMPAT_GET_ROT(pose));
  truth.pose.orientation.y = GZ_COMPAT_GET_Y(GZ_COMPAT_GET_ROT(pose));
  truth.pose.orientation.z = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_ROT(pose));
  truth.pose.position.x = GZ_COMPAT_GET_X(GZ_COMPAT_GET_POS(pose));
  truth.pose.position.y = GZ_COMPAT_GET_Y(GZ_COMPAT_GET_POS(pose));
  truth.pose.position.z = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(pose));
  truth.twist.linear.x = GZ_COMPAT_GET_X(vel);
  truth.twist.linear.y = GZ_COMPAT_GET_Y(vel);
  truth.twist.linear.z = GZ_COMPAT_GET_Z(vel);
  truth.twist.angular.x = GZ_COMPAT_GET_X(omega);
  truth.twist.angular.y = GZ_COMPAT_GET_Y(omega);
  truth.twist.angular.z = GZ_COMPAT_GET_Z(omega);
  truth.acceleration.linear.x = GZ_COMPAT_GET_X(lin_accel);
  truth.acceleration.linear.y = GZ_COMPAT_GET_Y(lin_accel);
  truth.acceleration.linear.z = GZ_COMPAT_GET_Z(lin_accel);
  truth.acceleration.angular.x = GZ_COMPAT_GET_X(ang_accel);
  truth.acceleration.angular.y = GZ_COMPAT_GET_Y(ang_accel);
  truth.acceleration.angular.z = GZ_COMPAT_GET_Z(ang_accel);

  // Convert to NED from NWU
  // truth.header.frame_id = link_name_ + "_NED";
  truth.pose.orientation.y *= -1.0;
  truth.pose.orientation.z *= -1.0;
  truth.pose.position.y *= -1.0;
  truth.pose.position.z *= -1.0;
  truth.twist.linear.y *= -1.0;
  truth.twist.linear.z *= -1.0;
  truth.twist.angular.y *= -1.0;
  truth.twist.angular.z *= -1.0;
  truth.acceleration.linear.y *= -1.0;
  truth.acceleration.linear.z *= -1.0;
  truth.acceleration.angular.y *= -1.0;
  truth.acceleration.angular.z *= -1.0;

  return truth;
}

geometry_msgs::msg::Vector3Stamped GazeboDynamics::compute_wind_truth()
{
  geometry_msgs::msg::Vector3Stamped current_wind;
  current_wind.header.stamp = this->get_clock()->now();
  // TODO: Publish the wind in the inertial frame
  return current_wind;
}

bool GazeboDynamics::set_sim_state(const rosflight_msgs::msg::SimState state)
{
  // TODO: Check that this rotation is correct direction.
  GazeboPose new_pose{state.pose.position.x,
                      state.pose.position.y,
                      state.pose.position.z,
                      state.pose.orientation.w,
                      state.pose.orientation.x,
                      state.pose.orientation.y,
                      state.pose.orientation.z};
  GZ_COMPAT_SET_WORLD_COG_POSE(link_, new_pose);

  GazeboVector lin_vel{state.twist.linear.x, state.twist.linear.y, state.twist.linear.z};
  GazeboVector ang_vel{state.twist.angular.x, state.twist.angular.y, state.twist.angular.z};
  GZ_COMPAT_SET_WORLD_LINEAR_VEL(link_, lin_vel);
  GZ_COMPAT_SET_ANGULAR_VEL(link_, ang_vel);

  // Can't add accels directly... Have to set forces and torques that produce those accels.
  GazeboVector lin_acc{state.acceleration.linear.x, state.acceleration.linear.y, state.acceleration.linear.z};
  GazeboVector ang_acc{state.acceleration.angular.x, state.acceleration.angular.y, state.acceleration.angular.z};

  // Parse the inertia parameters
  double mass = link_->GetInertial()->Mass();
  GazeboMatrix3 I_body = link_->GetInertial()->MOI();

  // Get world rotation of the link
  GazeboQuaternion q = GZ_COMPAT_GET_ROT(GZ_COMPAT_GET_WORLD_COG_POSE(link_));
  GazeboMatrix3 R(q);
  GazeboMatrix3 I_world = R * I_body * R.Transposed();

  GazeboVector force = mass * lin_acc;
  GazeboVector torque = I_world  * ang_acc;

  GZ_COMPAT_ADD_COG_FORCE(link_, force);
  GZ_COMPAT_ADD_COG_TORQUE(link_, torque);

  return true;
}

} // namespace rosflight_sim
