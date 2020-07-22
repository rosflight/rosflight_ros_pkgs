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

#include <rosflight/mavrosflight/logger_adapter.h>
#include <rosflight/mavrosflight/logger_interface.h>
#include <rosflight/mavrosflight/time_manager.h>

namespace mavrosflight
{
template <typename DerivedLogger>
TimeManager<DerivedLogger>::TimeManager(MavlinkComm *comm, LoggerInterface<DerivedLogger> &logger) :
  comm_(comm),
  offset_alpha_(0.95),
  offset_ns_(0),
  offset_(0.0),
  initialized_(false),
  logger_(logger)

{
  comm_->register_mavlink_listener(this);

  ros::NodeHandle nh;
  time_sync_timer_ = nh.createTimer(ros::Duration(ros::Rate(10)), &TimeManager::timer_callback, this);
}

template <typename DerivedLogger>
void TimeManager<DerivedLogger>::handle_mavlink_message(const mavlink_message_t &msg)
{
  int64_t now_ns = ros::Time::now().toNSec();

  if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC)
  {
    mavlink_timesync_t tsync;
    mavlink_msg_timesync_decode(&msg, &tsync);

    if (tsync.tc1 > 0) // check that this is a response, not a request
    {
      int64_t offset_ns = (tsync.ts1 + now_ns - 2 * tsync.tc1) / 2;

      if (!initialized_ || std::abs(offset_ns_ - offset_ns) > 1e7) // if difference > 10ms, use it directly
      {
        offset_ns_ = offset_ns;
        logger_.info("Detected time offset of %0.3f s.", offset_ns / 1e9);
        logger_.debug("FCU time: %0.3f, System time: %0.3f", tsync.tc1 * 1e-9, tsync.ts1 * 1e-9);
        initialized_ = true;
      }
      else // otherwise low-pass filter the offset
      {
        offset_ns_ = offset_alpha_ * offset_ns + (1.0 - offset_alpha_) * offset_ns_;
      }
    }
  }
}

template <typename DerivedLogger>
ros::Time TimeManager<DerivedLogger>::get_ros_time_ms(uint32_t boot_ms)
{
  if (!initialized_)
    return ros::Time::now();

  int64_t boot_ns = (int64_t)boot_ms * 1000000;

  int64_t ns = boot_ns + offset_ns_;
  if (ns < 0)
  {
    logger_.error_throttle(1, "negative time calculated from FCU: boot_ns=%ld, offset_ns=%ld.  Using system time",
                           boot_ns, offset_ns_);
    return ros::Time::now();
  }
  ros::Time now;
  now.fromNSec(ns);
  return now;
}

template <typename DerivedLogger>
ros::Time TimeManager<DerivedLogger>::get_ros_time_us(uint64_t boot_us)
{
  if (!initialized_)
    return ros::Time::now();

  int64_t boot_ns = (int64_t)boot_us * 1000;

  int64_t ns = boot_ns + offset_ns_;
  if (ns < 0)
  {
    logger_.error_throttle(1, "negative time calculated from FCU: boot_ns=%ld, offset_ns=%ld.  Using system time",
                           boot_ns, offset_ns_);
    return ros::Time::now();
  }
  ros::Time now;
  now.fromNSec(ns);
  return now;
}

template <typename DerivedLogger>
void TimeManager<DerivedLogger>::timer_callback(const ros::TimerEvent &event)
{
  mavlink_message_t msg;
  mavlink_msg_timesync_pack(1, 50, &msg, 0, ros::Time::now().toNSec());
  comm_->send_message(msg);
}

template class TimeManager<DerivedLoggerType>;

} // namespace mavrosflight
