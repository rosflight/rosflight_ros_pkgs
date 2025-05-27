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

#ifndef ROSFLIGHT_SIM_STANDALONE_DYNAMICS_H
#define ROSFLIGHT_SIM_STANDALONE_DYNAMICS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rosflight_sim/dynamics_interface.hpp"
#include "rosflight_msgs/srv/set_sim_state.hpp"

namespace rosflight_sim
{

class StandaloneDynamics : public DynamicsInterface
{
public:
  StandaloneDynamics();

private:
  void apply_forces_and_torques(const geometry_msgs::msg::WrenchStamped & forces_torques) override;
  rosflight_msgs::msg::SimState compute_truth() override;
  geometry_msgs::msg::Vector3Stamped compute_wind_truth() override;

  // Persistent variables
  rosflight_msgs::msg::SimState current_truth_state_;
  geometry_msgs::msg::Vector3Stamped current_wind_truth_;
  Eigen::Matrix3d J_;
  Eigen::Matrix3d J_inv_;

  /**
  * @brief Equations of motion for the system.
  * x_dot = f(x,u)
  */
  Eigen::VectorXd f(Eigen::VectorXd state, Eigen::VectorXd forces);

  void rk4(Eigen::VectorXd state, Eigen::VectorXd forces_moments, double dt);
  double compute_dt(double now);

  void compute_inertia_matrix();
  Eigen::VectorXd add_gravity_forces(Eigen::VectorXd forces);
  Eigen::VectorXd add_ground_collision_forces(Eigen::VectorXd forces);

  void declare_parameters();

  // Service definitions used for initialization
  rclcpp::Service<rosflight_msgs::srv::SetSimState>::SharedPtr set_sim_state_srvs_;
  bool set_sim_state(const rosflight_msgs::srv::SetSimState::Request::SharedPtr req,
                     const rosflight_msgs::srv::SetSimState::Response::SharedPtr res);
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_STANDALONE_DYNAMICS_H
