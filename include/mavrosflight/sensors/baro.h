/**
 * \file imu.h
 * \author James Jackson <superjax08@gmail.com>
 */

#ifndef MAVROSFLIGHT_SENSORS_BARO_H
#define MAVROSFLIGHT_SENSORS_BARO_H

#include <mavrosflight/mavlink_bridge.h>

namespace mavrosflight
{
namespace sensors
{

/**
 * \brief Barometer sensor class
 */
class Baro
{
public:

  Baro(double alpha, double ground, int settling_count, int calibration_count);
  Baro();

  // The Alpha in the LPF of the barometer
  double alt_alpha_;

  // The position of the ground in the barometer
  double alt_ground_;

  /**
   * \brief Get corrected measurement values
   * \param msg The raw barometer message
   * \param[out] alt The altitude in meters
   * \return True if the measurement is valid
   */
  bool correct(mavlink_small_baro_t msg, double *alt);

private:
  // calibration variables
  int calibration_counter_;
  double calibration_sum_;
  int settling_count_; // settle for a second or so
  int calibration_count_;

  // offsets and filters
  double prev_alt_;

};

} // namespace sensors
} // namespace mavrosflight

#endif // MAVROSFLIGHT_SENSORS_IMU_H
