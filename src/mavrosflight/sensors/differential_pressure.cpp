/**
 * \file diff_pressure.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/sensors/differential_pressure.h>

namespace mavrosflight
{
namespace sensors
{

DifferentialPressure::DifferentialPressure() :
  calibration_counter_(0),
  calibration_count_(100),
  diff_press_offset_(0.0)
{}

bool DifferentialPressure::correct(mavlink_diff_pressure_t msg, double *pressure, double *temperature)
{
  // conversion from pixhawk source code
  *temperature = ((200.0f * msg.temperature) / 2047) - 50;

  /*
   * this equation is an inversion of the equation in the
   * pressure transfer function figure on page 4 of the datasheet
   * We negate the result so that positive differential pressures
   * are generated when the bottom port is used as the static
   * port on the pitot and top port is used as the dynamic port
   */
  double diff_press_PSI = -((msg.diff_pressure - 0.1*16383) * (P_MAX-P_MIN)/(0.8*16383) + P_MIN);
  double diff_press_pa_raw = diff_press_PSI * PSI_TO_PA;
  if (calibration_counter_ > calibration_count_)
  {
    *pressure = diff_press_pa_raw - diff_press_offset_;
    return true;
  }
  else if (calibration_counter_ == calibration_count_)
  {
    diff_press_offset_ = diff_press_offset_/calibration_count_;
    calibration_counter_++;
    return false;
  }
  else
  {
    diff_press_offset_ += diff_press_pa_raw;
    calibration_counter_++;
    return false;
  }
}



} // namespace sensors
} // namespace mavrosflight
