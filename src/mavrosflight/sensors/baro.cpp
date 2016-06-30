/**
 * \file baro.cpp
 * \author James Jackson <superjax08@gmail.com>
 */

#include <mavrosflight/sensors/baro.h>
#include <ros/ros.h>

namespace mavrosflight
{
namespace sensors
{

Baro::Baro(double alpha, double ground, int settling_count, int calibration_count)
{
  Baro();
  alt_alpha_ = alpha;
  alt_ground_ = ground;
  settling_count_ = settling_count; // settle for a second or so
  calibration_count_ = calibration_count;
}

Baro::Baro()
{
  alt_alpha_ = 0.3;
  alt_ground_ = 0.0;
  settling_count_ = 20; // settle for a second or so
  calibration_count_ = 20;
  calibration_counter_ = 0;
  calibration_sum_ = 0;
  prev_alt_ = 0.0;
}

bool Baro::correct(mavlink_small_baro_t baro, double *alt)
{
  double pressure = baro.pressure;
  double temperature = baro.temperature;

  if( calibration_counter_ > calibration_count_ + settling_count_)
  {
    double alt_tmp = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 4430.0f; // in meters

    // offset calculated ground altitude
    alt_tmp -= alt_ground_;

    // LPF measurements
    *alt = alt_alpha_*alt_tmp + (1.0 - alt_alpha_)*prev_alt_;
    prev_alt_ = *alt;
  }
  if (calibration_counter_ < settling_count_)
  {
    calibration_counter_++;
  }
  else if (calibration_counter_ < settling_count_ + calibration_count_)
  {
    double measurement = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 4430.0f;
    calibration_sum_ += measurement;
    calibration_counter_++;
  }
  else if(calibration_counter_ == settling_count_ + calibration_count_)
  {
    alt_ground_ = calibration_sum_/calibration_count_;
    ROS_INFO_STREAM("BARO CALIBRATED " << alt_ground_ << " meters above sea level");
    calibration_counter_++;
  }
}


} // namespace sensors
} // namespace mavrosflight
