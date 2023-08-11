/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
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

#ifndef ROSFLIGHT_SIM_MAV_FORCES_AND_MOMENTS_H
#define ROSFLIGHT_SIM_MAV_FORCES_AND_MOMENTS_H

#include <eigen3/Eigen/Core>

namespace rosflight_sim
{
/**
 * @brief Base class for forces and moments classes for UAVs.
 */
class MAVForcesAndMoments
{
protected:
  /**
   * @brief Saturation function for actuator commands.
   *
   * @param x Unsaturated command
   * @param max Max allowable command value
   * @param min Min allowable command value
   * @return Saturated command
   */
  static double sat(double x, double max, double min)
  {
    if (x > max) {
      return max;
    } else if (x < min) {
      return min;
    } else {
      return x;
    }
  }

  /**
   * @brief Determine the largest value between two possible values.
   *
   * @param x Value to compare
   * @param y Value to compare
   * @return x or y, whichever is largest
   */
  static double max(double x, double y) { return (x > y) ? x : y; }

public:
  /**
   * @brief Struct for storing the position and velocity of the MAV for both translation and rotation
   * coordinates, as well as the time of the state.
   */
  struct CurrentState
  {
    /// Position of MAV in NED wrt initial position
    Eigen::Vector3d pos;
    /// Rotation of MAV in NED wrt initial position
    Eigen::Matrix3d rot;
    /// Body-fixed velocity of MAV wrt initial position (NED)
    Eigen::Vector3d vel;
    /// Body-fixed angular velocity of MAV (NED)
    Eigen::Vector3d omega;
    /// current time
    double t;
  };

  /**
   * @brief Interface function for calculating the current MAV forces and moments for Gazebo.
   *
   * @param x Current state of MAV
   * @param act_cmds Current MAV commands
   * @return Calculated forces and moments
   */
  virtual Eigen::Matrix<double, 6, 1> update_forces_and_torques(CurrentState x,
                                                                const int act_cmds[]) = 0;
  /**
   * @brief Interface function for updating the wind speed to use in the forces and moments calculations.
   *
   * @param wind Wind velocities
   */
  virtual void set_wind(Eigen::Vector3d wind) = 0;
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_MAV_FORCES_AND_MOMENTS_H
