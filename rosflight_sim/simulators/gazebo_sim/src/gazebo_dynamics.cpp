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

#include "gazebo_sim/include/gazebo_dynamics.hpp"

namespace rosflight_sim
{

GazeboDynamics::GazeboDynamics() {}

void GazeboDynamics::publish_truth()
{
  GazeboPose pose = GZ_COMPAT_GET_WORLD_COG_POSE(link_);
  GazeboVector vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  GazeboVector omega = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);

  // Publish truth
  nav_msgs::msg::Odometry truth;
  truth.header.stamp.sec = GZ_COMPAT_GET_SIM_TIME(world_).sec;
  truth.header.stamp.nanosec = GZ_COMPAT_GET_SIM_TIME(world_).nsec;
  truth.header.frame_id = link_name_ + "_NWU";
  truth.pose.pose.orientation.w = GZ_COMPAT_GET_W(GZ_COMPAT_GET_ROT(pose));
  truth.pose.pose.orientation.x = GZ_COMPAT_GET_X(GZ_COMPAT_GET_ROT(pose));
  truth.pose.pose.orientation.y = GZ_COMPAT_GET_Y(GZ_COMPAT_GET_ROT(pose));
  truth.pose.pose.orientation.z = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_ROT(pose));
  truth.pose.pose.position.x = GZ_COMPAT_GET_X(GZ_COMPAT_GET_POS(pose));
  truth.pose.pose.position.y = GZ_COMPAT_GET_Y(GZ_COMPAT_GET_POS(pose));
  truth.pose.pose.position.z = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(pose));
  truth.twist.twist.linear.x = GZ_COMPAT_GET_X(vel);
  truth.twist.twist.linear.y = GZ_COMPAT_GET_Y(vel);
  truth.twist.twist.linear.z = GZ_COMPAT_GET_Z(vel);
  truth.twist.twist.angular.x = GZ_COMPAT_GET_X(omega);
  truth.twist.twist.angular.y = GZ_COMPAT_GET_Y(omega);
  truth.twist.twist.angular.z = GZ_COMPAT_GET_Z(omega);
  truth_NWU_pub_->publish(truth);

  // Convert to NED
  truth.header.frame_id = link_name_ + "_NED";
  truth.pose.pose.orientation.y *= -1.0;
  truth.pose.pose.orientation.z *= -1.0;
  truth.pose.pose.position.y *= -1.0;
  truth.pose.pose.position.z *= -1.0;
  truth.twist.twist.linear.y *= -1.0;
  truth.twist.twist.linear.z *= -1.0;
  truth.twist.twist.angular.y *= -1.0;
  truth.twist.twist.angular.z *= -1.0;
  // TODO: move this up an inheritance level
  truth_NED_pub_->publish(truth);
}

// TODO: What should we do with this?
void GazeboDynamics::wind_callback(const geometry_msgs::msg::Vector3 & msg)
{
  Eigen::Vector3d wind;
  wind << msg.x, msg.y, msg.z;
  mav_dynamics_->set_wind(wind);
}


} // namespace rosflight_sim
