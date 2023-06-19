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

#pragma GCC diagnostic ignored "-Wwrite-strings"

#include <cstdint>
#include <cstdio>
#include <sstream>

#include <eigen3/Eigen/Core>

#include <rosflight_sim/rosflight_sil.h>

namespace rosflight_sim
{
ROSflightSIL::ROSflightSIL() : gazebo::ModelPlugin(), comm_(board_), firmware_(board_, comm_) {}

ROSflightSIL::~ROSflightSIL()
{
  GZ_COMPAT_DISCONNECT_WORLD_UPDATE_BEGIN(updateConnection_);
}

void ROSflightSIL::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  node_ = gazebo_ros::Node::Get(_sdf);
  model_ = _model;
  world_ = model_->GetWorld();

  DeclareROSParams();

  /*
   * Connect the Plugin to the Robot and Save pointers to the various elements in the simulation
   */
  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[ROSflight_SIL] Please specify a linkName of the forces and moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[ROSflight_SIL] Couldn't find specified link \"" << link_name_ << "\".");

  /* Load Params from Gazebo Server */
  if (_sdf->HasElement("mavType"))
  {
    mav_type_ = _sdf->GetElement("mavType")->Get<std::string>();
  }
  else
  {
    mav_type_ = "multirotor";
    gzerr << "[rosflight_sim] Please specify a value for parameter \"mavType\".\n";
  }

  if (mav_type_ == "multirotor")
    mav_dynamics_ = new Multirotor(node_);
  else if (mav_type_ == "fixedwing")
    mav_dynamics_ = new Fixedwing(node_);
  else
    gzthrow("unknown or unsupported mav type\n");

  // Initialize the Firmware
  board_.gazebo_setup(link_, world_, model_, node_, mav_type_);
  firmware_.init();

  // Connect the update function to the simulation
  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ROSflightSIL::OnUpdate, this, _1));

  initial_pose_ = GZ_COMPAT_GET_WORLD_COG_POSE(link_);

  truth_NED_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("truth/NED", 1);
  truth_NWU_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("truth/NWU", 1);
}

void ROSflightSIL::DeclareROSParams() {
  // Declare multirotor parameters
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

  node_->declare_parameter("ground_altitude", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("gyro_stdev", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("gyro_bias_range", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("gyro_bias_walk_stdev", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("acc_stdev", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("acc_bias_range", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("acc_bias_walk_stdev", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("baro_stdev", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("baro_bias_range", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("baro_bias_walk_stdev", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("sonar_stdev", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("sonar_min_range", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("sonar_max_range", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("airspeed_stdev", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("airspeed_bias_range", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("airspeed_bias_walk_stdev", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("mag_stdev", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("mag_bias_range", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("mag_bias_walk_stdev", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("inclination", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("declination", rclcpp::PARAMETER_DOUBLE);

  // Declare fixedwing parameters
  node_->declare_parameter("Jx", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("Jy", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("Jz", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("Jxz", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("rho", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("wing_s", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("wing_b", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("wing_c", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("wing_M", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("wing_epsilon", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("wing_alpha0", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("k_motor", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("k_T_P", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("k_Omega", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("prop_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("prop_S", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("prop_C", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_L_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_L_delta_r", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_D_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_D_delta_r", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_ell_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_ell_delta_r", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_m_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_m_delta_r", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_n_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_n_delta_r", rclcpp::PARAMETER_DOUBLE);

  node_->declare_parameter("C_Y_O", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_alpha", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_beta", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_p", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_q", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_r", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_delta_a", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_delta_e", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("C_Y_delta_r", rclcpp::PARAMETER_DOUBLE);
}

// This gets called by the world update event.
void ROSflightSIL::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
  // We run twice so that that functions that take place when we don't have new IMU data get run
  firmware_.run();
  firmware_.run();

  Eigen::Matrix3d NWU_to_NED;
  NWU_to_NED << 1, 0, 0, 0, -1, 0, 0, 0, -1;

  MAVForcesAndMoments::Current_State state;
  GazeboPose pose = GZ_COMPAT_GET_WORLD_COG_POSE(link_);
  GazeboVector vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  GazeboVector omega = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);

  // Convert gazebo types to Eigen and switch to NED frame
  state.pos = NWU_to_NED * vec3_to_eigen_from_gazebo(GZ_COMPAT_GET_POS(pose));
  state.rot = NWU_to_NED * rotation_to_eigen_from_gazebo(GZ_COMPAT_GET_ROT(pose));
  state.vel = NWU_to_NED * vec3_to_eigen_from_gazebo(vel);
  state.omega = NWU_to_NED * vec3_to_eigen_from_gazebo(omega);
  state.t = _info.simTime.Double();

  forces_ = mav_dynamics_->updateForcesAndTorques(state, board_.get_outputs());

  // apply the forces and torques to the joint (apply in NWU)
  GazeboVector force = vec3_to_gazebo_from_eigen(NWU_to_NED * forces_.block<3, 1>(0, 0));
  GazeboVector torque = vec3_to_gazebo_from_eigen(NWU_to_NED * forces_.block<3, 1>(3, 0));
  link_->AddRelativeForce(force);
  link_->AddRelativeTorque(torque);

  publishTruth();
}

void ROSflightSIL::Reset()
{
  link_->SetWorldPose(initial_pose_);
  link_->ResetPhysicsStates();
  //  start_time_us_ = (uint64_t)(world_->GetSimTime().Double() * 1e3);
  //  rosflight_init();
}

void ROSflightSIL::windCallback(const geometry_msgs::msg::Vector3& msg)
{
  Eigen::Vector3d wind;
  wind << msg.x, msg.y, msg.z;
  mav_dynamics_->set_wind(wind);
}

void ROSflightSIL::publishTruth()
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
  truth_NED_pub_->publish(truth);
}

Eigen::Vector3d ROSflightSIL::vec3_to_eigen_from_gazebo(GazeboVector vec)
{
  Eigen::Vector3d out;
  out << GZ_COMPAT_GET_X(vec), GZ_COMPAT_GET_Y(vec), GZ_COMPAT_GET_Z(vec);
  return out;
}

GazeboVector ROSflightSIL::vec3_to_gazebo_from_eigen(Eigen::Vector3d vec)
{
  GazeboVector out(vec(0), vec(1), vec(2));
  return out;
}

Eigen::Matrix3d ROSflightSIL::rotation_to_eigen_from_gazebo(GazeboQuaternion quat)
{
  Eigen::Quaterniond eig_quat(GZ_COMPAT_GET_W(quat), GZ_COMPAT_GET_X(quat), GZ_COMPAT_GET_Y(quat),
                              GZ_COMPAT_GET_Z(quat));
  return eig_quat.toRotationMatrix();
}

GZ_REGISTER_MODEL_PLUGIN(ROSflightSIL)
} // namespace rosflight_sim
