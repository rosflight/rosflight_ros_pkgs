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
 * \file baro.h
 * \author James Jackson <superjax08@gmail.com>
 */

#ifndef MAVROSFLIGHT_SENSORS_BARO_H
#define MAVROSFLIGHT_SENSORS_BARO_H

#include <rosflight/mavrosflight/mavlink_bridge.h>

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
