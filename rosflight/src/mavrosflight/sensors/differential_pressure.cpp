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
 * \file diff_pressure.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <rosflight/mavrosflight/sensors/differential_pressure.h>

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
