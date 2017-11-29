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
 * \file mavrosflight.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVROSFLIGHT_H
#define MAVROSFLIGHT_MAVROSFLIGHT_H

#include <rosflight/mavrosflight/mavlink_bridge.h>
#include <rosflight/mavrosflight/mavlink_comm.h>
#include <rosflight/mavrosflight/param_manager.h>
#include <rosflight/mavrosflight/time_manager.h>

#include <rosflight/mavrosflight/mavlink_listener_interface.h>
#include <rosflight/mavrosflight/param_listener_interface.h>

#include <boost/function.hpp>

#include <stdint.h>
#include <string>

namespace mavrosflight
{

class MavROSflight
{
public:

  /**
   * \brief Instantiates the class and begins communication on the specified serial port
   * \param mavlink_comm Reference to a MavlinkComm object (serial or UDP)
   * \param baud_rate Serial communication baud rate
   */
  MavROSflight(MavlinkComm& mavlink_comm, uint8_t sysid = 1, uint8_t compid = 50);

  /**
   * \brief Stops communication and closes the serial port before the object is destroyed
   */
  ~MavROSflight();

  // public member objects
  MavlinkComm& comm;
  ParamManager param;
  TimeManager time;

private:

  // member variables
  uint8_t sysid_;
  uint8_t compid_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_MAVROSFLIGHT_H
