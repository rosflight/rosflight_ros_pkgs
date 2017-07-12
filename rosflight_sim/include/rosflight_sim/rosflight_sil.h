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

#include <stdio.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include <rosflight.h>
#include <rosflight_sim/sil_board.h>

#include <rosflight_sim/mav_forces_and_moments.h>
#include <rosflight_sim/multirotor_forces_and_moments.h>
#include <rosflight_sim/fixedwing_forces_and_moments.h>

namespace rosflight_sim
{

class ROSflightSIL : public gazebo::ModelPlugin {
public:
  ROSflightSIL();
  ~ROSflightSIL();

protected:
  void Reset();
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const gazebo::common::UpdateInfo & /*_info*/);

private:
  SIL_Board board_;
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

  MAVForcesAndMoments* mav_dynamics_;

  // container for forces
  MAVForcesAndMoments::ForcesAndTorques forces_;
  MAVForcesAndMoments::ForcesAndTorques applied_forces_;

  // Time Counters
  double sampling_time_;
  double prev_sim_time_;
  uint64_t start_time_us_;

  ros::NodeHandle* nh_;

  // For reset handling
  gazebo::math::Pose initial_pose_;
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_ROSFLIGHT_SIL_H
