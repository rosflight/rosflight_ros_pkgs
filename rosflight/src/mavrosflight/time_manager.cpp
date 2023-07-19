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
 * \file time_manager.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */
#include <functional>

#include <rosflight/mavrosflight/time_manager.hpp>

namespace mavrosflight
{
TimeManager::TimeManager(MavlinkComm * const comm, rclcpp::Node * const node)
    : comm_(comm), node_(node), offset_alpha_(0.95), offset_ns_(0), initialized_(false)
{
  comm_->register_mavlink_listener(this);
  time_sync_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&TimeManager::timer_callback, this), nullptr);
}

void TimeManager::handle_mavlink_message(const mavlink_message_t & msg)
{
  std::chrono::nanoseconds now(node_->get_clock()->now().nanoseconds());

  if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
    mavlink_timesync_t tsync;
    mavlink_msg_timesync_decode(&msg, &tsync);

    std::chrono::nanoseconds tc1_chrono(tsync.tc1);

    if (tsync.tc1 > 0) // check that this is a response, not a request
    {
      std::chrono::nanoseconds ts1_chrono(tsync.ts1);
      std::chrono::nanoseconds offset_ns((ts1_chrono + now - 2 * tc1_chrono) / 2);

      // if difference > 10ms, use it directly
      if (!initialized_ || (offset_ns_ - offset_ns) > std::chrono::milliseconds(10)
        || (offset_ns_ - offset_ns) < std::chrono::milliseconds(-10))
      {
        RCLCPP_INFO(node_->get_logger(), "Detected time offset of %0.3f s.",
                    abs(std::chrono::duration<double>(offset_ns_ - offset_ns).count()));
        RCLCPP_DEBUG(node_->get_logger(), "FCU time: %0.3f, System time: %0.3f", tsync.tc1 * 1e-9,
                     tsync.ts1 * 1e-9);
        offset_ns_ = offset_ns;
        initialized_ = true;
      } else // otherwise low-pass filter the offset
      {
        offset_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
          offset_alpha_ * offset_ns + (1.0 - offset_alpha_) * offset_ns_);
      }
    }
  }
}

std::chrono::nanoseconds TimeManager::fcu_time_to_system_time(std::chrono::nanoseconds fcu_time)
{
  if (!initialized_) { return std::chrono::nanoseconds(node_->get_clock()->now().nanoseconds()); }

  std::chrono::nanoseconds ns = fcu_time + offset_ns_;
  if (ns < std::chrono::nanoseconds::zero()) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1,
      "negative time calculated from FCU: fcu_time=%ld, offset_ns=%ld.  Using system time",
      fcu_time.count(), offset_ns_.count());
    return std::chrono::nanoseconds(node_->get_clock()->now().nanoseconds());
  }
  return ns;
}

void TimeManager::timer_callback()
{
  mavlink_message_t msg;
  mavlink_msg_timesync_pack(1, 50, &msg, 0, node_->get_clock()->now().nanoseconds());
  comm_->send_message(msg);
}

} // namespace mavrosflight
