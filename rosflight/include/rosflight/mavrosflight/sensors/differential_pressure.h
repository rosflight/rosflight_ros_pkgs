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
 * \file diff_pressure.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_SENSORS_DIFF_PRESSURE_H
#define MAVROSFLIGHT_SENSORS_DIFF_PRESSURE_H

#include <rosflight/mavrosflight/mavlink_bridge.h>

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

} // namespace sensors
} // namespace mavrosflight

#endif // MAVROSFLIGHT_SENSORS_DIFF_PRESSURE_H
