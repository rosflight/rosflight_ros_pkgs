/*
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
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

/**
 * \file imu.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 * \author James Jackson <me@jamessjackson.com>
 */

#ifndef MAVROSFLIGHT_SENSORS_IMU_H
#define MAVROSFLIGHT_SENSORS_IMU_H

#include <rosflight/mavrosflight/mavlink_bridge.h>
#include <eigen3/Eigen/Core>
#include <deque>

namespace mavrosflight
{
namespace sensors
{

/**
 * \brief IMU sensor class
 */
class Imu
{
public:

  Imu();

  /**
   * \brief Begin the temperature calibration routine
   */
  void start_temp_calibration();

  /**
   * \brief Check if a temperature calibration is in progress
   * \return True if a temperature calibration is currently in progress
   */
  bool is_calibrating() { return calibrating_; }

  /**
   * \brief Calibrate the IMU for temperature and bias compensation
   * \param msg The raw IMU message
   * \return True if the calibration is done
   */
  bool calibrate_temp(mavlink_small_imu_t msg);

  /**
   * \brief Get corrected measurement values
   * \param msg The raw IMU message
   * \param[out] xacc The accelerometer X value (m/s^2)
   * \param[out] yacc The accelerometer Y value (m/s^2)
   * \param[out] zacc The accelerometer Z value (m/s^2)
   * \param[out] xgyro The rate gyro X value (rad/s)
   * \param[out] ygyro The rate gyro Y value (rad/s)
   * \param[out] zgyro The rate gyro Z value (rad/s)
   * \return True if the measurement is valid
   */
  bool correct(mavlink_small_imu_t msg,
               double *xacc, double *yacc, double *zacc, double *xgyro, double *ygyro, double *zgyro, double *temperature);

  /// These are the publicly available versions of the accel calibration
  /// The const stuff is to make it read-only
  const double xm() const { return x_[0](0); }
  const double ym() const { return x_[1](0); }
  const double zm() const { return x_[2](0); }
  const double xb() const { return x_[0](1); }
  const double yb() const { return x_[1](1); }
  const double zb() const { return x_[2](1); }

private:
  Eigen::Vector2d x_[3];

  bool calibrating_; //!< whether a temperature calibration is in progress
  double calibration_time_; //!< seconds to record data for temperature compensation
  double deltaT_; //!< number of degrees required for a temperature calibration
  double Tmin_; //!< minimum temperature seen
  double Tmax_; //!< maximum temperature seen
  bool first_time_; //!< waiting for first measurement for calibration
  double start_time_; //!< timestamp of first calibration measurement
  int measurement_throttle_;
  std::deque<double> A_;
  std::deque<Eigen::Vector3d> B_;
};

} // namespace sensors
} // namespace mavrosflight

#endif // MAVROSFLIGHT_SENSORS_IMU_H
