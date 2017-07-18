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
 * \file calibrate_accel_temp.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 * \author James Jackson <me@jamessjackson.com>
 */

#include <rosflight/calibrate_accel_temp.h>

#include <boost/bind.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace rosflight
{

CalibrateAccelTemp::CalibrateAccelTemp() :
  calibrating_(false),
  deltaT_(1.0),
  nh_private_("~"),
  imu_subscriber_(nh_, "imu/data", 1),
  temp_subscriber_(nh_, "imu/temperature", 1),
  synchronizer_(imu_subscriber_, temp_subscriber_, 10)
{
  x_[0] << 0, 0;
  x_[1] << 0, 0;
  x_[2] << 0, 0;

  calibration_time_ = nh_private_.param<double>("calibration_time", 60.0);
  measurement_skip_ = nh_private_.param<int>("measurement_skip", 20);

  param_set_client_ = nh_.serviceClient<rosflight_msgs::ParamSet>("param_set");
  synchronizer_.registerCallback(boost::bind(&CalibrateAccelTemp::imu_callback, this, _1, _2));
}

void CalibrateAccelTemp::run()
{
  // reset calibration parameters
  bool success = true;
  success = success && set_param("ACC_X_BIAS", 0.0);
  success = success && set_param("ACC_Y_BIAS", 0.0);
  success = success && set_param("ACC_Z_BIAS", 0.0);

  success = success && set_param("ACC_X_TEMP_COMP", 0.0);
  success = success && set_param("ACC_Y_TEMP_COMP", 0.0);
  success = success && set_param("ACC_Z_TEMP_COMP", 0.0);

  if (!success)
  {
    ROS_FATAL("Failed to reset calibration parameters");
    return;
  }

  // collect data
  ROS_WARN("Calibrating IMU, hold flight controller level and still for %g seconds!", calibration_time_);
  start_temp_calibration();
  while (calibrating_)
  {
    ros::spinOnce();
  }

  // compute calibration
  do_temp_calibration();

  // set calibration parameters
  success = true;
  success = success && set_param("ACC_X_BIAS", xb());
  success = success && set_param("ACC_Y_BIAS", yb());
  success = success && set_param("ACC_Z_BIAS", zb());

  success = success && set_param("ACC_X_TEMP_COMP", xm());
  success = success && set_param("ACC_Y_TEMP_COMP", ym());
  success = success && set_param("ACC_Z_TEMP_COMP", zm());

  if (!success)
  {
    ROS_FATAL("Failed to set calibration parameters");
    return;
  }

  ROS_INFO("Calibration complete");
  ROS_WARN("Write parameters to save calibration!");
}

void CalibrateAccelTemp::start_temp_calibration()
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

void CalibrateAccelTemp::do_temp_calibration()
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

    // put the data into an Eigen Matrix for linear algebra
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
}

bool CalibrateAccelTemp::imu_callback(const sensor_msgs::Imu::ConstPtr &imu,
                                      const sensor_msgs::Temperature::ConstPtr &temp)
{
  if (calibrating_)
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
      if (measurement_throttle_ > measurement_skip_)
      {
        Eigen::Vector3d measurement;
        measurement << imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z - 9.80665;

        A_.push_back(temp->temperature);
        B_.push_back(measurement);

        if (temp->temperature < Tmin_)
        {
          Tmin_ = temp->temperature;
        }
        if (temp->temperature > Tmax_)
        {
          Tmax_ = temp->temperature;
        }
        measurement_throttle_ = 0;
      }
      measurement_throttle_++;
    }
    else
    {
      calibrating_ = false;
    }
  }
}

bool CalibrateAccelTemp::set_param(std::string name, double value)
{
  rosflight_msgs::ParamSet srv;
  srv.request.name = name;
  srv.request.value = value;

  if (param_set_client_.call(srv))
  {
    return srv.response.exists;
  }
  else
  {
    return false;
  }
}

} // namespace rosflight
