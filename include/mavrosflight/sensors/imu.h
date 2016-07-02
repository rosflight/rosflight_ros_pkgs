/**
 * \file imu.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_SENSORS_IMU_H
#define MAVROSFLIGHT_SENSORS_IMU_H

#include <mavrosflight/mavlink_bridge.h>
#include <eigen3/Eigen/Core>

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

  bool calibrated;

  /**
   * \brief Calibrate the IMU for temperature and bias compensation
   * \param msg The raw IMU message
   * \return True if the calibration is done
   */
  bool calibrate(mavlink_small_imu_t msg);

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
  const double xm() const {return x_[0](0);}
  const double ym() const {return x_[1](0);}
  const double zm() const {return x_[2](0);}
  const double xb() const {return x_[0](1);}
  const double yb() const {return x_[1](1);}
  const double zb() const {return x_[2](1);}

private:
  //! \todo explicitly compute these so it's clear where they come from
  static const double ACCEL_SCALE = 0.002349;
  static const double GYRO_SCALE = 0.004256;
  Eigen::Vector2d x_[3];
  double calibration_time_;
};

} // namespace sensors
} // namespace mavrosflight

#endif // MAVROSFLIGHT_SENSORS_IMU_H
