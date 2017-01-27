/**
 * \file diff_pressure.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_SENSORS_DIFF_PRESSURE_H
#define MAVROSFLIGHT_SENSORS_DIFF_PRESSURE_H

#include <mavrosflight/mavlink_bridge.h>

namespace mavrosflight
{
namespace sensors
{
class DifferentialPressure
{
public:
  DifferentialPressure();

  /**
   * \brief Get corrected measurement values
   * \param msg The raw differential pressure message
   * \param[out] pressure The pressure (Pa)
   * \param[out] temperature The temperature (deg C)
   * \return True if the measurement is valid
   */
  bool correct(mavlink_diff_pressure_t msg, double *pressure, double *temperature);

private:
  static const double P_MIN = -1.0;
  static const double P_MAX = 1.0;
  static const double PSI_TO_PA = 6894.757;

  int calibration_counter_;
  int calibration_count_;
  double diff_press_offset_;
};

}  // namespace sensors
}  // namespace mavrosflight

#endif  // MAVROSFLIGHT_SENSORS_DIFF_PRESSURE_H
