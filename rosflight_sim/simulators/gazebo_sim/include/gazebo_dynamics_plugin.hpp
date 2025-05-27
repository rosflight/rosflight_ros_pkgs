/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2024 Jacob Moore
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


#ifndef ROSFLIGHT_SIM_GAZEBO_PLUGIN_H
#define ROSFLIGHT_SIM_GAZEBO_PLUGIN_H

#include <thread>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

#include "gz_compat.hpp"
#include "gazebo_dynamics.hpp"

namespace rosflight_sim
{

/**
 * @brief This class serves as a plugin for Gazebo, interacting directly with Gazebo.
 *
 * @note This class does not inherit from rclcpp::Node as Gazebo provides the node that should be
 * used.
 */
class GazeboDynamicsPlugin : public gazebo::ModelPlugin
{
public:
  GazeboDynamicsPlugin();
  ~GazeboDynamicsPlugin() override;

  gazebo::physics::WorldPtr get_world() { return world_; }
  gazebo::physics::ModelPtr get_model() { return model_; }
  gazebo::physics::LinkPtr get_link() { return link_; }
  rclcpp::Node::SharedPtr get_node() { return node_;}

  Eigen::Vector3d vec3_to_eigen_from_gazebo(GazeboVector vec);
  GazeboVector vec3_to_gazebo_from_eigen(Eigen::Vector3d vec);
  Eigen::Matrix3d rotation_to_eigen_from_gazebo(GazeboQuaternion quat);

protected:
  /**
   * @brief Determines what should happen when the reset action is called within Gazebo.
   */
  void Reset() override;
  /**
   * @brief Initializes the plugin. Should only be called once, on Gazebo startup.
   */
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  /**
   * @brief Callback function used by Gazebo to update the simulation.
   */
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

private:
  std::string mav_type_;
  std::string namespace_;
  std::string link_name_;

  // For reset handling
  GazeboPose initial_pose_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::EntityPtr parent_link_;
  gazebo::event::ConnectionPtr updateConnection_; // Pointer to the update event connection.
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<rosflight_sim::GazeboDynamics> gazebo_dynamics_ptr_;
  std::thread spin_thread_;
};

} // rosflight_sim

#endif // ROSFLIGHT_SIM_GAZEBO_SENSOR_PLUGIN_H
