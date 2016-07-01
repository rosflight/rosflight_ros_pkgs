/**
 * \file imu.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 * \author James Jackson <me@jamessjackson.com>
 */

#include <mavrosflight/sensors/imu.h>
#include <ros/ros.h>
#include <deque>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace mavrosflight
{
namespace sensors
{

Imu::Imu()
{
  x_[0] << 0, 0;
  x_[1] << 0, 0;
  x_[2] << 0, 0;
  calibrated = true;
}

bool Imu::calibrate(mavlink_small_imu_t msg)
{
  // read temperature of accel chip for temperature compensation calibration
  double temperature = ((double)msg.temperature/340.0 + 36.53);

  static double calibration_time = 5.0; // seconds to record data for temperature compensation
  static double deltaT = 1.0; // number of degrees required for a temperature calibration

  static double Tmin = 1000; // minimum temperature seen
  static double Tmax = -1000; // maximum temperature seen

  // one for each accel axis
  static std::deque<Eigen::Vector3d> A(0);
  static std::deque<double> B(0);
  static bool first_time = true;

  static double start_time = 0;

  if(first_time)
  {
    first_time = false;
    start_time = ros::Time::now().toSec();
  }

  double now = ros::Time::now().toSec() - start_time;

  // if still in calibration mode
  if(now < calibration_time || (Tmax - Tmin) < deltaT)
  {
    static int measurement_throttle = 0;
    if(measurement_throttle > 20)
    {
      Eigen::Vector3d measurement;
      measurement << msg.xacc, msg.yacc, msg.zacc - 4096; // need a better way to know the z-axis offset
      A.push_back(measurement);
      B.push_back(temperature);
      if(temperature < Tmin)
      {
        Tmin = temperature;
      }
      if(temperature > Tmax)
      {
        Tmax = temperature;
      }
      measurement_throttle = 0;
    }
    measurement_throttle++;
  }
  else if(!calibrated)
  {
    for(int i = 0; i < 3; i++)
    {
      Eigen::MatrixX2d Amat;
      Eigen::VectorXd Bmat;
      Amat.resize(A.size(),2);
      Bmat.resize(B.size());

      ROS_INFO("IMU DONE CALIBRATING, current time = %f, start = %f", now, calibration_time);
      // put the data into and Eigen Matrix for linear algebra
      for(int j = 0; j<A.size(); j++)
      {
        Amat(j,0) = A[j](i);
        Amat(j,1) = 1.0;
        Bmat(j) = B[j];
      }

      // Perform Least-Squares on the data
      x_[i] = Amat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(Bmat);
    }
    calibrated = true;
  }
  return calibrated;
}

bool Imu::correct(mavlink_small_imu_t msg,
                  double *xacc, double *yacc, double *zacc, double *xgyro, double *ygyro, double *zgyro, double *temp)
{
  *xacc = msg.xacc * ACCEL_SCALE;
  *yacc = msg.yacc * ACCEL_SCALE;
  *zacc = msg.zacc * ACCEL_SCALE;

  *xgyro = msg.xgyro * GYRO_SCALE;
  *ygyro = msg.ygyro * GYRO_SCALE;
  *zgyro = msg.zgyro * GYRO_SCALE;

  *temp = msg.temperature/340.0 + 36.53;
  return true;
}



} // namespace sensors
} // namespace mavrosflight
