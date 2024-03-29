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
 * \file time_manager.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_TIME_MANAGER_H
#define MAVROSFLIGHT_TIME_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <rosflight_io/mavrosflight/mavlink_bridge.hpp>
#include <rosflight_io/mavrosflight/mavlink_comm.hpp>
#include <rosflight_io/mavrosflight/mavlink_listener_interface.hpp>

#include <chrono>
#include <memory>

namespace mavrosflight
{
class TimeManager : MavlinkListenerInterface
{
public:
  TimeManager(MavlinkComm * comm, rclcpp::Node * node);

  void handle_mavlink_message(const mavlink_message_t & msg) override;

  std::chrono::nanoseconds fcu_time_to_system_time(std::chrono::nanoseconds fcu_time);

private:
  MavlinkComm * const comm_;
  rclcpp::Node * const node_;

  rclcpp::TimerBase::SharedPtr time_sync_timer_;
  void timer_callback();

  double offset_alpha_;
  std::chrono::nanoseconds offset_ns_;

  bool initialized_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_TIME_MANAGER_H
