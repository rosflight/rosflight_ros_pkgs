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
 * \file calibrate_accel_temp.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 * \author James Jackson <me@jamessjackson.com>
 */

#ifndef ROSFLIGHT_SENSORS_CALIBRATE_IMU_H
#define ROSFLIGHT_SENSORS_CALIBRATE_IMU_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <rosflight_msgs/ParamSet.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

#include <eigen3/Eigen/Core>

#include <deque>

namespace rosflight
{

/**
 * \brief IMU sensor class
 */
class CalibrateAccelTemp
{
public:

  CalibrateAccelTemp();

  void run();

  /**
   * \brief Begin the temperature calibration routine
   */
  void start_temp_calibration();

  void do_temp_calibration();

  /**
   * \brief Calibrate the IMU for temperature and bias compensation
   * \param msg The raw IMU message
   * \return True if the calibration is done
   */
  bool imu_callback(const sensor_msgs::Imu::ConstPtr& imu, const sensor_msgs::Temperature::ConstPtr& temp);

  /// These are the publicly available versions of the accel calibration
  /// The const stuff is to make it read-only
  const double xm() const { return x_[0](0); }
  const double ym() const { return x_[1](0); }
  const double zm() const { return x_[2](0); }
  const double xb() const { return x_[0](1); }
  const double yb() const { return x_[1](1); }
  const double zb() const { return x_[2](1); }

private:
  bool set_param(std::string name, double value);

  Eigen::Vector2d x_[3];

  bool calibrating_; //!< whether a temperature calibration is in progress
  double calibration_time_; //!< seconds to record data for temperature compensation
  double deltaT_; //!< number of degrees required for a temperature calibration
  double Tmin_; //!< minimum temperature seen
  double Tmax_; //!< maximum temperature seen
  bool first_time_; //!< waiting for first measurement for calibration
  double start_time_; //!< timestamp of first calibration measurement
  int measurement_skip_; //!< the number of measurements to skip
  int measurement_throttle_;
  std::deque<double> A_;
  std::deque<Eigen::Vector3d> B_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  message_filters::Subscriber<sensor_msgs::Imu> imu_subscriber_; //!< subscriber for the IMU messages
  message_filters::Subscriber<sensor_msgs::Temperature> temp_subscriber_; //!< subscriber for the temperature messages
  message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::Temperature> synchronizer_; //!< synchronizer for the IMU and temperature messages

  ros::ServiceClient param_set_client_;
};

} // namespace rosflight

#endif // ROSFLIGHT_SENSORS_CALIBRATE_IMU_H
