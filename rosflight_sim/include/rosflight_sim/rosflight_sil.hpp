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

#ifndef ROSFLIGHT_SIM_ROSFLIGHT_SIL_H
#define ROSFLIGHT_SIM_ROSFLIGHT_SIL_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mavlink/mavlink.h>
#include <rosflight.h>
#include <rosflight_sim/sil_board.hpp>

#include <rosflight_sim/fixedwing_forces_and_moments.hpp>
#include <rosflight_sim/mav_forces_and_moments.hpp>
#include <rosflight_sim/multirotor_forces_and_moments.hpp>

#include <rosflight_sim/gz_compat.hpp>

namespace rosflight_sim
{
/**
 * @brief This class serves as the SIL plugin for Gazebo, holding an instance of sil_board,
 * calling the forces and moments calculation functions, and interacting directly with Gazebo.
 *
 * @note This class does not inherit from rclcpp::Node as Gazebo provides the node that should be
 * used.
 */
class ROSflightSIL : public gazebo::ModelPlugin
{
public:
  ROSflightSIL();
  ~ROSflightSIL() override;

protected:
  /**
   * @brief Determines what should happen when the reset action is called within Gazebo.
   */
  void Reset() override;
  /**
   * @brief Initializes the SIL. Should only be called once, on Gazebo startup.
   */
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  /**
   * @brief Callback function used by Gazebo to update the simulation. Contains forces and moments
   * calculations.
   */
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

private:
  void wind_callback(const geometry_msgs::msg::Vector3 & msg);
  /**
   * @brief Callback for publishing true pose on ROS topic using information obtained directly from
   * Gazebo.
   */
  void publish_truth();

  rclcpp::Node::SharedPtr node_;

  SILBoard board_;
  rosflight_firmware::Mavlink comm_;
  rosflight_firmware::ROSflight firmware_;

  std::string mav_type_;
  std::string namespace_;
  std::string link_name_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::EntityPtr parent_link_;
  gazebo::event::ConnectionPtr updateConnection_; // Pointer to the update event connection.

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_NED_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_NWU_pub_;

  MAVForcesAndMoments * mav_dynamics_;

  // container for forces
  Eigen::Matrix<double, 6, 1> forces_, applied_forces_;

  // For reset handlin
  GazeboPose initial_pose_;

  /**
   * @brief Converts a vector from a gazebo datatype to an eigen datatype.
   */
  static Eigen::Vector3d vec3_to_eigen_from_gazebo(GazeboVector vec);
  /**
   * @brief Converts a vector from an eigen datatype to a gazebo datatype.
   */
  static GazeboVector vec3_to_gazebo_from_eigen(Eigen::Vector3d vec);
  /**
   * @brief Create an eigen rotation matrix from a Gazebo quaternion.
   */
  static Eigen::Matrix3d rotation_to_eigen_from_gazebo(GazeboQuaternion vec);

  /**
   * Declares all ROS params to be used by the class. Must be called in the constructor.
   */
  void declare_SIL_params();
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_ROSFLIGHT_SIL_H
