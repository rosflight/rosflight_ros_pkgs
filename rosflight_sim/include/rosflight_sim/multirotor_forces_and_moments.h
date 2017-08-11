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

#ifndef ROSFLIGHT_SIM_MULTIROTOR_FORCES_AND_MOMENTS_H
#define ROSFLIGHT_SIM_MULTIROTOR_FORCES_AND_MOMENTS_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

#include <rosflight_sim/mav_forces_and_moments.h>

namespace rosflight_sim
{

class Multirotor : public MAVForcesAndMoments {
private:
    ros::NodeHandle* nh_;
    Eigen::Vector3d wind_;

    double prev_time_;

    struct Rotor{
      double max;
      std::vector<double> F_poly;
      std::vector<double> T_poly;
      double tau_up; // time constants for response
      double tau_down;
    };

    struct Motor{
      Rotor rotor;
      Eigen::Vector3d position;
      Eigen::Vector3d normal;
      int direction; // 1 for CW -1 for CCW
    };

    int num_rotors_;
    std::vector<Motor> motors_;

    double linear_mu_;
    double angular_mu_;
    std::vector<double> ground_effect_;

    double mass_;

    // Container for an Actuator
    struct Actuator{
      double max;
      double tau_up;
      double tau_down;
    };

    // Struct of Actuators
    // This organizes the physical limitations of the abstract torques and Force
    struct Actuators{
      Actuator l;
      Actuator m;
      Actuator n;
      Actuator F;
    } actuators_;

    Eigen::MatrixXd rotor_position_;
    Eigen::MatrixXd rotor_plane_normal_;
    Eigen::VectorXd rotor_rotation_direction_;

    Eigen::MatrixXd force_allocation_matrix_;
    Eigen::MatrixXd torque_allocation_matrix_;
    Eigen::VectorXd desired_forces_;
    Eigen::VectorXd desired_torques_;
    Eigen::VectorXd actual_forces_;
    Eigen::VectorXd actual_torques_;

public:
    Multirotor(ros::NodeHandle* nh);
    ~Multirotor();

    Eigen::Matrix<double, 6, 1> updateForcesAndTorques(Current_State x, const int act_cmds[]);
    void set_wind(Eigen::Vector3d wind);
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_MULTIROTOR_FORCES_AND_MOMENTS_H
