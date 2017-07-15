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

#include <rosflight_sim/rosflight_sil.h>
#include <rosflight_sim/sil_board.h>

namespace rosflight_sim
{

ROSflightSIL::ROSflightSIL() :
  gazebo::ModelPlugin(),
  nh_(nullptr),
  prev_sim_time_(0),
  firmware_(board_)
{
}

ROSflightSIL::~ROSflightSIL()
{
  gazebo::event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void ROSflightSIL::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin");
    return;
  }
  ROS_INFO("Loaded the ROSflight SIL plugin");

  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();

  /*
   * Connect the Plugin to the Robot and Save pointers to the various elements in the simulation
   */
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[ROSflight_SIL] Please specify a namespace.\n";
  nh_ = new ros::NodeHandle(namespace_);

  gzmsg << "loading parameters from " << namespace_ << " ns\n";

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[ROSflight_SIL] Please specify a linkName of the forces and moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[ROSflight_SIL] Couldn't find specified link \"" << link_name_ << "\".");

  /* Load Params from Gazebo Server */
  if (_sdf->HasElement("mavType")) {
    mav_type_ = _sdf->GetElement("mavType")->Get<std::string>();
  }
  else {
    mav_type_ = "multirotor";
    gzerr << "[rosflight_sim] Please specify a value for parameter \"mavType\".\n";
  }

  if(mav_type_ == "multirotor")
    mav_dynamics_ = new Multirotor(nh_);
  else if(mav_type_ == "fixedwing")
    mav_dynamics_ = new Fixedwing(nh_);
  else
    gzthrow("unknown or unsupported mav type\n");

   // Initialize the Firmware
   board_.gazebo_setup(link_, world_, model_, nh_, mav_type_);
   board_.init_board();
   firmware_.init();

  // Connect the update function to the simulation
  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ROSflightSIL::OnUpdate, this, _1));

  initial_pose_ = link_->GetWorldCoGPose();
}


// This gets called by the world update event.
void ROSflightSIL::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();

  MAVForcesAndMoments::Pose pos;
  gazebo::math::Pose W_pose_W_C = link_->GetWorldCoGPose();
  pos.pn = W_pose_W_C.pos.x; // We should check to make sure that this is right
  pos.pe = -W_pose_W_C.pos.y;
  pos.pd = -W_pose_W_C.pos.z;
  gazebo::math::Vector3 euler_angles = W_pose_W_C.rot.GetAsEuler();
  pos.phi = euler_angles.x;
  pos.theta = -euler_angles.y;
  pos.psi = -euler_angles.z;

  MAVForcesAndMoments::Velocities vel;
  gazebo::math::Vector3 C_linear_velocity_W_C = link_->GetRelativeLinearVel();
  vel.u = C_linear_velocity_W_C.x;
  vel.v = -C_linear_velocity_W_C.y;
  vel.w = -C_linear_velocity_W_C.z;
  gazebo::math::Vector3 C_angular_velocity_W_C = link_->GetRelativeAngularVel();
  vel.p = C_angular_velocity_W_C.x;
  vel.q = -C_angular_velocity_W_C.y;
  vel.r = -C_angular_velocity_W_C.z;

  firmware_.run();

  forces_ = mav_dynamics_->updateForcesAndTorques(pos, vel, board_.get_outputs(), sampling_time_);

  // apply the forces and torques to the joint
  link_->AddRelativeForce(gazebo::math::Vector3(forces_.Fx, -forces_.Fy, -forces_.Fz));
  link_->AddRelativeTorque(gazebo::math::Vector3(forces_.l, -forces_.m, -forces_.n));
}

void ROSflightSIL::Reset()
{
    link_->SetWorldPose(initial_pose_);
    link_->ResetPhysicsStates();
//  start_time_us_ = (uint64_t)(world_->GetSimTime().Double() * 1e3);
//  rosflight_init();
}

GZ_REGISTER_MODEL_PLUGIN(ROSflightSIL)

} // namespace rosflight_sim
