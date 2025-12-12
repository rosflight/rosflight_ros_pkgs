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
  , J_{Eigen::Matrix3d::Identity()}
  , J_inv_{Eigen::Matrix3d::Identity()}
  , inertia_matrix_changed_{false}
  , wind_params_changed_{false}
  , dt_{0.0}
  , prev_time_{0.0}
  , steady_state_wind_{Eigen::Vector3d::Zero()}
{
  current_truth_state_ = rosflight_msgs::msg::SimState();

  // Declare parameters and set up the callback for changing parameters
  declare_parameters();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&StandaloneDynamics::parameters_callback, this, std::placeholders::_1));

  sample_ = std::mt19937(std::random_device{}());
  normal_dist_ = std::normal_distribution<double>();

  compute_inertia_matrix();
  set_steady_state_wind();
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

  this->declare_parameter("sim_has_wind", false);
  this->declare_parameter("use_random_wind", true);
  this->declare_parameter("steady_state_wind", std::vector<double>{0.0, 0.0, 0.0});

  this->declare_parameter("random_wind_mean", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("random_wind_std_dev", std::vector<double>{0.5, 0.5, 0.0});

  this->declare_parameter("wind_has_gusts", false);
  this->declare_parameter("interoccurance_time_gust_s", 5.0);
  this->declare_parameter("gust_magnitude_mean", 0.15);
  this->declare_parameter("gust_magnitude_std_dev", 0.3);
  this->declare_parameter("gust_duration_limits", std::vector<double>{0.5, 5.0});
  this->declare_parameter("gust_dir_std_dev", 0.1);
}

rcl_interfaces::msg::SetParametersResult
StandaloneDynamics::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result = DynamicsInterface::parameters_callback(parameters);
  if (!result.successful) {
    return result;
  }

  for (const rclcpp::Parameter& param : parameters) {
    if (param.get_name() == "Jxx"
        || param.get_name() == "Jxy"
        || param.get_name() == "Jxz"
        || param.get_name() == "Jyy"
        || param.get_name() == "Jyz"
        || param.get_name() == "Jzz") {
      inertia_matrix_changed_ = true;
    } else if (param.get_name() == "sim_has_wind"
               || param.get_name() == "use_random_wind"
               || param.get_name() == "steady_state_wind"
               || param.get_name() == "random_wind_mean"
               || param.get_name() == "random_wind_std_dev") {
      wind_params_changed_ = true;
    }
  }

  return result;
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

void StandaloneDynamics::set_steady_state_wind()
{
  steady_state_wind_ = Eigen::Vector3d(0.0,0.0,0.0);

  if (!this->get_parameter("sim_has_wind").as_bool()) {
    return;
  }

  auto wind_params = this->get_parameter("steady_state_wind").as_double_array();

  if (!this->get_parameter("use_random_wind").as_bool()) {
    steady_state_wind_ = Eigen::Vector3d(wind_params[0], wind_params[1], wind_params[2]);
  }
  else {
    std::vector<double> wind_means = this->get_parameter("random_wind_mean").as_double_array();
    std::vector<double> wind_std_devs = this->get_parameter("random_wind_std_dev").as_double_array();
    steady_state_wind_ = Eigen::Vector3d(wind_means[0] + normal_dist_(sample_)*wind_std_devs[0],
                                      wind_means[1] + normal_dist_(sample_)*wind_std_devs[1],
                                      wind_means[2] + normal_dist_(sample_)*wind_std_devs[2]);
  }
}

void StandaloneDynamics::apply_forces_and_torques(const geometry_msgs::msg::WrenchStamped & forces_torques)
{
  compute_dt(forces_torques.header.stamp.sec + forces_torques.header.stamp.nanosec * 1e-9);
  if (dt_ == 0.0) { return; } // Don't integrate if the timestep is zero

  if (inertia_matrix_changed_) {
    compute_inertia_matrix();
    inertia_matrix_changed_ = false;
  }

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
  rk4(state, fm);
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
  if (current_truth_state_.pose.position.z < -0.001) { // inertial NED frame
    // If far from ground, do nothing
    return forces;
  }

  current_truth_state_.pose.position.z = std::min(0.0, current_truth_state_.pose.position.z);

  Eigen::Quaterniond q_body_to_inertial{current_truth_state_.pose.orientation.w, 
                                        current_truth_state_.pose.orientation.x,
                                        current_truth_state_.pose.orientation.y,
                                        current_truth_state_.pose.orientation.z};
  Eigen::Vector3d forces_in_inertial_frame = q_body_to_inertial * forces.segment(0,3);

  if (forces_in_inertial_frame(2) > 0.0) {
    forces_in_inertial_frame(2) = 0.0; // If down force is positive, set it to zero.
  }

  forces.segment(0,3) = q_body_to_inertial.inverse() * forces_in_inertial_frame;
  return forces;
}

void StandaloneDynamics::rk4(Eigen::VectorXd state, Eigen::VectorXd forces_moments)
{
  // RK4
  Eigen::VectorXd k1 = f(state, forces_moments);
  Eigen::VectorXd k2 = f(state + dt_/2 * k1, forces_moments);
  Eigen::VectorXd k3 = f(state + dt_/2 * k2, forces_moments);
  Eigen::VectorXd k4 = f(state + dt_ * k3, forces_moments);
  state += dt_ / 6 * (k1 + 2*k2 + 2*k3 + k4); // Integrate states

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

void StandaloneDynamics::compute_dt(double now)
{
  if(prev_time_ == 0) {
    prev_time_ = now;
    return;
  }

  // Calculate time
  dt_ = now - prev_time_;
  prev_time_ = now;
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
  if (wind_params_changed_) {
    set_steady_state_wind();
    wind_params_changed_ = false;
  }

  // Wind in the inertial frame
  current_wind_truth_.header.stamp = this->now();
  current_wind_truth_.vector.x = steady_state_wind_[0];
  current_wind_truth_.vector.y = steady_state_wind_[1];
  current_wind_truth_.vector.z = steady_state_wind_[2];

  if (!this->get_parameter("wind_has_gusts").as_bool() || dt_ == 0.0) {
    return current_wind_truth_;
  }

  Eigen::Vector3d steady_state_wind_dir = steady_state_wind_.normalized();

  // Conduct Bernoulli trial.
  // lambda * dt
  float bernoulli_probability = dt_/this->get_parameter("interoccurance_time_gust_s").as_double();
  std::bernoulli_distribution is_gust(bernoulli_probability);

  // If the Bernoulli trial is successful add a gust.
  if (is_gust(sample_)) {
    // Normally distributed but stricly positive gust magnitudes
    float gust_mag = this->get_parameter("gust_magnitude_mean").as_double()
                                         + this->get_parameter("gust_magnitude_std_dev").as_double()*normal_dist_(sample_);
    
    // Using small angle, this is roughly the deviation in radians
    float std_dev_gust_dir = this->get_parameter("gust_dir_std_dev").as_double();

    // Normally distributed direction of the gust centered on steady state wind direction.
    Eigen::Vector3d gust_dir;
    gust_dir[0] = steady_state_wind_dir[0] + std_dev_gust_dir*normal_dist_(sample_);
    gust_dir[1] = steady_state_wind_dir[1] + std_dev_gust_dir*normal_dist_(sample_);
    gust_dir[2] = steady_state_wind_dir[2] + std_dev_gust_dir*normal_dist_(sample_);
    gust_dir = gust_dir.normalized();
    
    // Uniformily distributed duration of the gust.
    std::vector<double> gust_duration_limits = this->get_parameter("gust_duration_limits").as_double_array();
    std::uniform_real_distribution gust_duration_dist(gust_duration_limits[0], gust_duration_limits[1]);

    double gust_duration = gust_duration_dist(sample_);
    
    // Add gust to queue of gusts.
    Gust gust_to_add{gust_dir, gust_mag, gust_duration, 0.};
    gusts_.push_back(gust_to_add);
  }

  Eigen::Vector3d cumulative_gust(0.0,0.0,0.0);
  
  // Compute the effect of each gust.
  for (Gust& gust : gusts_) {
    cumulative_gust += compute_gust(gust);
  }

  current_wind_truth_.vector.x += cumulative_gust[0];
  current_wind_truth_.vector.y += cumulative_gust[1];
  current_wind_truth_.vector.z += cumulative_gust[2];
  
  // Remove gust from queue if it has completed its duration.
  gusts_.erase(
    std::remove_if( gusts_.begin(), gusts_.end(),
                   [](Gust gust) -> bool
                   {return gust.t > gust.duration; }),
                   gusts_.end());

  return current_wind_truth_;
}

Eigen::Vector3d StandaloneDynamics::compute_gust(Gust& gust) {
  // This equation comes from Discrete Gust Model for Launch Vehicle Assessments by Frank B. Leahy (NASA, 2008).
  // It models a wind gust as a 1-cosine wave with a variable magnitude and duration.
  
  float cur_mag = gust.magintude/2.0 * (1-cosf(2*M_PI/gust.duration * gust.t));
  gust.t += dt_;

  return gust.gust_dir * cur_mag;
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
