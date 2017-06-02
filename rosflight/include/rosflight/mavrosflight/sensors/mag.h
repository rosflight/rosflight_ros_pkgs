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
 * \file mag.h
 * \author Jerel Nielsen <jerel.nielsen@gmail.com>
 */

#ifndef MAVROSFLIGHT_SENSORS_MAG_H
#define MAVROSFLIGHT_SENSORS_MAG_H

#include <rosflight/mavrosflight/mavlink_bridge.h>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <random>
#include <deque>

namespace mavrosflight
{
namespace sensors
{

/**
 * \brief MAG sensor class
 */
class Mag
{
public:

  Mag();

  /**
   * \brief Begin the temperature calibration routine
   */
  void start_calibration();

  /**
   * @brief set_refence_magnetic_field_strength
   * @param reference_magnetic_field
   */
  void set_refence_magnetic_field_strength(double reference_magnetic_field);

  /**
   * \brief Check if a calibration is in progress
   * \return True if a calibration is currently in progress
   */
  bool is_calibrating() { return calibrating_; }

  /**
   * \brief Calibrate the magnetometer for hard and soft iron effects
   * \param msg The raw MagneticField message
   * \return True if the calibration is done
   */
  bool calibrate(mavlink_small_mag_t msg);

  /**
   * \brief Get corrected measurement values
   * \param msg The raw MagneticField message
   * \param[out] xmag The magnetometer X value (Tesla)
   * \param[out] ymag The magnetometer Y value (Tesla)
   * \param[out] zmag The magnetometer Z value (Tesla)
   * \return True if the measurement is valid
   */
  bool correct(mavlink_small_mag_t msg, double *xmag, double *ymag, double *zmag);

  /// These are the publicly available versions of the mag calibration
  /// The const stuff is to make it read-only
  const double a11() const { return A_(0, 0); }
  const double a12() const { return A_(0, 1); }
  const double a13() const { return A_(0, 2); }
  const double a21() const { return A_(1, 0); }
  const double a22() const { return A_(1, 1); }
  const double a23() const { return A_(1, 2); }
  const double a31() const { return A_(2, 0); }
  const double a32() const { return A_(2, 1); }
  const double a33() const { return A_(2, 2); }
  const double bx() const { return b_(0, 0); }
  const double by() const { return b_(1, 0); }
  const double bz() const { return b_(2, 0); }

private:
  Eigen::MatrixXd A_, b_;

  double reference_field_strength_; //!< the strength of earth's magnetic field at your location

  bool calibrating_; //!< whether a temperature calibration is in progress
  double calibration_time_; //!< seconds to record data for temperature compensation
  bool first_time_; //!< waiting for first measurement for calibration
  double start_time_; //!< timestamp of first calibration measurement
  double displayed_time_; //!< calibration time left already displayed
  int ransac_iters_; //!< number of ransac iterations to fit ellipsoid to mag measurements
  double inlier_thresh_; //!< threshold to consider measurement an inlier in ellipsoidRANSAC
  Eigen::Vector3d measurement_prev_;
  std::deque<Eigen::Vector3d> measurements_;

  // function to perform RANSAC on ellipsoid data
  Eigen::MatrixXd ellipsoidRANSAC(std::deque<Eigen::Vector3d> meas, int iters, double inlier_thresh);

  // function to vector from ellipsoid center to surface along input vector
  Eigen::Vector3d intersect(Eigen::Vector3d r_m, Eigen::Vector3d r_e, Eigen::MatrixXd Q, Eigen::MatrixXd ub, double k);

  /*
      sort eigenvalues and eigenvectors output from Eigen library
  */
  void eigSort(Eigen::MatrixXd &w, Eigen::MatrixXd &v);

  /*
      This function gets ellipsoid parameters via least squares on ellipsoidal data
      according to the paper: Li, Qingde, and John G. Griffiths. "Least squares ellipsoid
      specific fitting." Geometric modeling and processing, 2004. proceedings. IEEE, 2004.
  */
  Eigen::MatrixXd ellipsoidLS(std::deque<Eigen::Vector3d> meas);

  /*
      This function compute magnetometer calibration parameters according to Section 5.3 of the
      paper: Renaudin, Valérie, Muhammad Haris Afzal, and Gérard Lachapelle. "Complete triaxis
      magnetometer calibration in the magnetic domain." Journal of sensors 2010 (2010).
  */
  void magCal(Eigen::MatrixXd u, Eigen::MatrixXd &A, Eigen::MatrixXd &bb);
};

} // namespace sensors
} // namespace mavrosflight

#endif // MAVROSFLIGHT_SENSORS_MAG_H
