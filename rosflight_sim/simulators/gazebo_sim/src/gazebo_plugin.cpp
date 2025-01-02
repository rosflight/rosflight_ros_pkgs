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

#include "gazebo_sim/include/gazebo_plugin.hpp"

namespace rosflight_sim
{

GazeboPlugin::~GazeboPlugin() { GZ_COMPAT_DISCONNECT_WORLD_UPDATE_BEGIN(updateConnection_); }

void GazeboPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  node_ = gazebo_ros::Node::Get(_sdf);
  model_ = _model;
  world_ = model_->GetWorld();

  /*
   * Connect the Plugin to the Robot and Save pointers to the various elements in the simulation
   */
  if (_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    gzerr << "[ROSflight_SIL] Please specify a linkName of the forces and moments plugin.\n";
  }
  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr) {
    gzthrow("[ROSflight_SIL] Couldn't find specified link \"" << link_name_ << "\".")
  }

  /* Load Params from Gazebo Server */
  if (_sdf->HasElement("mavType")) {
    mav_type_ = _sdf->GetElement("mavType")->Get<std::string>();
  } else {
    mav_type_ = "multirotor";
    gzerr << "[rosflight_sim] Please specify a value for parameter \"mavType\".\n";
  }

  declare_SIL_params();

  if (mav_type_ == "multirotor") {
    mav_dynamics_ = new Multirotor(node_);
  } else if (mav_type_ == "fixedwing") {
    mav_dynamics_ = new Fixedwing(node_);
  } else {
    gzthrow("unknown or unsupported mav type\n")
  }

  // Initialize the Firmware
  board_.gazebo_setup(link_, world_, model_, node_, mav_type_);
  firmware_.init();

  // Connect the update function to the simulation
  updateConnection_ =
    gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ROSflightSIL::OnUpdate, this, _1));

  initial_pose_ = GZ_COMPAT_GET_WORLD_COG_POSE(link_);

  truth_NED_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("truth/NED", 1);
  truth_NWU_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("truth/NWU", 1);
}

} // rosflight_sim
