/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2025 Ian Reid, BYU MAGICC Lab.
 * All rights reserved. Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
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

/**
 * @file rosflight_io.h
 * @author Ian Reid <ian.young.reid\@gmail.com>
 */

#ifndef MAG_CALIBRATION_ROS_H
#define MAG_CALIBRATION_ROS_H

#include <unordered_set>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <string>
#include <random>

namespace rosflight_io
{
  enum class Orientation {
    X_DOWN,
    X_UP,
    Y_DOWN,
    Y_UP,
    Z_DOWN,
    Z_UP,
    NUM_ORIENTATIONS
  };

  enum class CalibrationState {
  REORIENTING,
  CHECKING,
  ALL_ORIENTATIONS_GATHERED,
  FINDING_CALIBRATION
};

/**
 * @class MagnetometerCalibrator
 * @brief Object that calculates the calibration values for the magneotometer.
 *
 * This uses the assumption that the correction matrix is symmetric and uses 
 * a least squares approach to fit an ellipsoid to the calibration data.
 */
class MagnetometerCalibrator
{
public:
  MagnetometerCalibrator(float accel_orientation_threshold, int consecutive_orientation_threshold, float lpf_alpha);

  void update_accel(Eigen::Vector3f accel);

  bool calibrating();

  void update_mag(double mag_x, double mag_y, double mag_z);

  std::string feedback();

  Eigen::Vector3d get_hard_iron_offset();
  
  Eigen::Matrix3d get_soft_iron_correction();

private:
  Eigen::VectorXd ellipsoid_least_squares(Eigen::MatrixXd data);

  void find_orientation();

  bool orientation_covered(Orientation orientation);

  bool all_orientation_data_gathered();

  void check_orientation_data();

  void calibrate();

  Eigen::Vector3d hard_iron_offset_;

  Eigen::Matrix3d soft_iron_correction_;

  float lpf_alpha_;

  Eigen::Vector3f lpf_accels_;

  Eigen::MatrixXd mag_calibration_data_;

  Orientation current_orientation_;

  int consecutive_orientation_;

  CalibrationState current_state_;

  float gravity_ = 9.81;

  float accel_orientation_threshold_;

  float consecutive_orientation_threshold_;

  std::unordered_set<Orientation> completed_orientations_;

  std::string feedback_;

  int start_of_current_orientation_data_;

  int num_times_data_checked_;

};

} // namespace rosflight_io

#endif // MAG_CALIBRATION_ROS_H
