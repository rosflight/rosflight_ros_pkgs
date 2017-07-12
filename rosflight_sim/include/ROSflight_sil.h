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


#ifndef ROSFLIGHT_SIL_H
#define ROSFLIGHT_SIL_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include "rosflight.h"
#include "SIL_board.h"

#include "mav_forces_and_moments.h"
#include "multirotor_forces_and_moments.h"
#include "fixedwing_forces_and_moments.h"

using namespace rosflight_sim;

namespace gazebo
{

class ROSflightSIL : public ModelPlugin {
public:
  ROSflightSIL();

  ~ROSflightSIL();

 // void SendForces();


protected:
  void UpdateForcesAndMoments();
  void UpdateEstimator();
  void Reset();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
  SIL_Board board_;
  rosflight_firmware::ROSflight firmware_;

  std::string mav_type_;
  std::string namespace_;
  std::string link_name_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  physics::EntityPtr parent_link_;
  event::ConnectionPtr updateConnection_; // Pointer to the update event connection.


  MAVForcesAndMoments* mav_dynamics_;

  // container for forces
  MAVForcesAndMoments::ForcesAndTorques forces_, applied_forces_;

  // Time Counters
  double sampling_time_;
  double prev_sim_time_;
  uint64_t start_time_us_;

  ros::NodeHandle* nh_;

  // For reset handling
  math::Pose initial_pose_;
};
}

#endif // ROSFLIGHT_SIL_H
