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
 * \file imu.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 * \author James Jackson <me@jamessjackson.com>
 */

#include <rosflight/mavrosflight/sensors/imu.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace mavrosflight
{
namespace sensors
{

Imu::Imu() :
  calibrating_(false),
  calibration_time_(60.0),
  deltaT_(1.0)
{
  x_[0] << 0, 0;
  x_[1] << 0, 0;
  x_[2] << 0, 0;
}

void Imu::start_temp_calibration()
{
  calibrating_ = true;

  Tmin_ = 1000;
  Tmax_ = -1000;

  first_time_ = true;
  start_time_ = 0;

  measurement_throttle_ = 0;

  B_.clear();
  A_.clear();
}

bool Imu::calibrate_temp(mavlink_small_imu_t msg)
{
  if (first_time_)
  {
    first_time_ = false;
    start_time_ = ros::Time::now().toSec();
  }

  double elapsed = ros::Time::now().toSec() - start_time_;

  // if still in calibration mode
  if (elapsed < calibration_time_ && (Tmax_ - Tmin_) < deltaT_)
  {
    if (measurement_throttle_ > 20)
    {
      Eigen::Vector3d measurement;
      measurement << msg.xacc, msg.yacc, msg.zacc - 9.80665;

      A_.push_back(msg.temperature);
      B_.push_back(measurement);

      if (msg.temperature < Tmin_)
      {
        Tmin_ = msg.temperature;
      }
      if (msg.temperature > Tmax_)
      {
        Tmax_ = msg.temperature;
      }
      measurement_throttle_ = 0;
    }
    measurement_throttle_++;
  }
  else if (calibrating_)
  {
    if ((Tmax_ - Tmin_) < deltaT_)
    {
      ROS_WARN("Insufficient temperature range (%f degC); calibration may be innaccurate!!!", Tmax_ - Tmin_);
    }

    for (int i = 0; i < 3; i++)
    {
      Eigen::MatrixX2d Amat;
      Eigen::VectorXd Bmat;
      Amat.resize(A_.size(),2);
      Bmat.resize(B_.size());

      // put the data into and Eigen Matrix for linear algebra
      std::deque<double>::iterator A_it = A_.begin();
      std::deque<Eigen::Vector3d>::iterator B_it = B_.begin();
      for (int j = 0; j < A_.size(); j++)
      {
        Amat(j,0) = *A_it;
        Amat(j,1) = 1.0;
        Bmat(j) = (*B_it)(i);
        B_it++;
        A_it++;
      }

      // Perform Least-Squares on the data
      x_[i] = Amat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(Bmat);
    }
    calibrating_ = false;
  }

  return !calibrating_;
}

bool Imu::correct(mavlink_small_imu_t msg,
                  double *xacc, double *yacc, double *zacc, double *xgyro, double *ygyro, double *zgyro, double *temp)
{
  *xacc = msg.xacc;
  *yacc = msg.yacc;
  *zacc = msg.zacc;

  *xgyro = msg.xgyro;
  *ygyro = msg.ygyro;
  *zgyro = msg.zgyro;

  *temp = msg.temperature;
  return true;
}

} // namespace sensors
} // namespace mavrosflight
