/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
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

/**
 * \file mag_cal.h
 * \author Jerel Nielsen <jerel.nielsen@gmail.com>
 * \author Devon Morris <devonmorris1992@gmail.com>
 */

#ifndef ROSFLIGHT_SENSORS_CALIBRATE_MAG_H
#define ROSFLIGHT_SENSORS_CALIBRATE_MAG_H

#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>

#include <rosflight_msgs/srv/param_set.hpp>

#include <sensor_msgs/msg/magnetic_field.hpp>

#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <random>

#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <vector>

namespace rosflight_io
{
/**
 * @class CalibrateMag sensor class
 * @brief Used to determine and set magnetometer calibration constants in firmware.
 *
 * This class is used to calibrate the magnetometer in the ROSflight firmware. It collects magnetometer
 * data, calculates the constants of best fit from that data, and then sets those parameters through
 * rosflight_io.
 */
class CalibrateMag : public rclcpp::Node
{
public:
  /**
   * @brief Default constructor for CalibrateMag.
   */
  CalibrateMag();

  /**
   * @brief Main run function for magnetometer calibration.
   */
  void run();

private:
  /**
   * @brief Begin the magnetometer calibration routine.
   */
  void start_mag_calibration();

  /**
   * @brief Calculate calibration constants from collected data.
   */
  void do_mag_calibration();

  /**
   * @brief Callback function for "magnetometer" ROS topic subscription.
   *
   * This function is called everytime CalibrateMag receives magnetometer data from rosflight_io over
   * the "magnetometer" topic. It collects magnetometer data until the calibration time has completed,
   * printing status messages in the process.
   *
   * @param mag ROS MagneticField message object.
   */
  bool mag_callback(const sensor_msgs::msg::MagneticField::ConstSharedPtr & mag);

  /// The const stuff is to make it read-only
  /// Get the value from A(1,1) (index starts at 1).
  double a11() const { return A_(0, 0); }
  /// Get the value from A(1,2) (index starts at 1).
  double a12() const { return A_(0, 1); }
  /// Get the value from A(1,3) (index starts at 1).
  double a13() const { return A_(0, 2); }
  /// Get the value from A(2,1) (index starts at 1).
  double a21() const { return A_(1, 0); }
  /// Get the value from A(2,2) (index starts at 1).
  double a22() const { return A_(1, 1); }
  /// Get the value from A(2,3) (index starts at 1).
  double a23() const { return A_(1, 2); }
  /// Get the value from A(3,1) (index starts at 1).
  double a31() const { return A_(2, 0); }
  /// Get the value from A(3,2) (index starts at 1).
  double a32() const { return A_(2, 1); }
  /// Get the value from A(3,3) (index starts at 1).
  double a33() const { return A_(2, 2); }
  /// Get the x value from the b matrix.
  double bx() const { return b_(0, 0); }
  /// Get the y value from the b matrix.
  double by() const { return b_(1, 0); }
  /// Get the z value from the b matrix.
  double bz() const { return b_(2, 0); }

  /**
   * @brief Set a ROSflight parameter.
   * @param name Name of parameter to set.
   * @param value Value to set parameter to.
   * @return Success of parameter setting.
   */
  bool set_param(std::string name, double value);

  /// "magnetometer" ROS topic subscription.
  message_filters::Subscriber<sensor_msgs::msg::MagneticField> mag_subscriber_;

  /// "param_set" ROS service client, used for setting ROSflight params.
  rclcpp::Client<rosflight_msgs::srv::ParamSet>::SharedPtr param_set_client_;

  Eigen::MatrixXd A_; ///< Matrix used in calibration constant calculation.
  Eigen::MatrixXd b_; ///< Matrix used in calibration constant calculation.

  double reference_field_strength_; ///< The strength of earth's magnetic field at your location.

  bool calibrating_;         ///< Flag for whether a calibration is currently in progress.
  bool first_time_;          ///< Flag for waiting for first measurement for calibration.
  double calibration_time_;  ///< Seconds to record data for calibration.
  double start_time_;        ///< Timestamp of first calibration measurement.
  int ransac_iters_;         ///< Number of ransac iterations to fit ellipsoid to mag measurements.
  int measurement_skip_;     ///< Number of measurements to skip at the start of calibration.
  int measurement_throttle_; ///< Stores the number measurements already skipped.
  double inlier_thresh_;     ///< Threshold to consider a measurement an inlier in ellipsoidRANSAC.
  Eigen::Vector3d measurement_prev_; ///< Stores previous measurement to ensure no duplicate measurements.
  EigenSTL::vector_Vector3d measurements_; ///< Stores all measurements.

  /**
   * @brief Function to perform RANSAC on ellipsoid data.
   * @param meas Vector of stored measurement data.
   * @param iters Number of iterations to run RANSAC on data.
   * @param inlier_thresh Distance threshold for which measurements are included in calibration.
   * @return Ellipsoid fit to measurements, for use in calibration.
   */
  Eigen::MatrixXd ellipsoidRANSAC(EigenSTL::vector_Vector3d meas, int iters, double inlier_thresh);

  /// Function to vector from ellipsoid center to surface along input vector
  static Eigen::Vector3d intersect(const Eigen::Vector3d & r_m, const Eigen::Vector3d & r_e,
                                   const Eigen::MatrixXd & Q, const Eigen::MatrixXd & ub, double k);

  /// Sort eigenvalues and eigenvectors output from Eigen library.
  static void eigSort(Eigen::MatrixXd & w, Eigen::MatrixXd & v);

  /**
   * @brief Gets ellipsoid parameters via least squares fitting.
   *
   * This function gets ellipsoid parameters via least squares on ellipsoidal data
   * according to the paper: Li, Qingde, and John G. Griffiths. "Least squares ellipsoid
   * specific fitting." Geometric modeling and processing, 2004. proceedings. IEEE, 2004.
   */
  static Eigen::MatrixXd ellipsoidLS(EigenSTL::vector_Vector3d meas);

  /**
   * @brief Compute magnetometer calibration parameters.
   *
   * This function compute magnetometer calibration parameters according to Section 5.3 of the
   * paper: Renaudin, Valérie, Muhammad Haris Afzal, and Gérard Lachapelle. "Complete triaxis
   * magnetometer calibration in the magnetic domain." Journal of sensors 2010 (2010).
   */
  void magCal(Eigen::MatrixXd u, Eigen::MatrixXd & A, Eigen::MatrixXd & bb) const;
};

} // namespace rosflight_io

#endif // ROSFLIGHT_SENSORS_CALIBRATE_MAG_H
