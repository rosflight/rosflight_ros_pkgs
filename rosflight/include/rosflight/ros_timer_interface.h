/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2020 Jacob Willis.
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file ros_timer_interface.h
 * @author Jacob Willis <jbwillis272@gmail.com>
 */

#ifndef ROSFLIGHT_ROS_TIMER_INTERFACE_H
#define ROSFLIGHT_ROS_TIMER_INTERFACE_H

#include <rosflight/mavrosflight/timer_interface.h>

#include <ros/ros.h>

#include <memory>
#include <vector>

namespace rosflight
{
class ROSAbstractTimer : public mavrosflight::AbstractTimer
{
public:
  inline ROSAbstractTimer(std::chrono::nanoseconds period,
                          std::function<void()> callback,
                          const bool oneshot,
                          const bool autostart) :
    callback_(callback)
  {
    ros::NodeHandle nh;
    ros_timer_ =
        nh.createTimer(ros::Duration(0, period.count()), &ROSAbstractTimer::callback_ros_, this, oneshot, autostart);
  }

  inline void start() { ros_timer_.start(); }

  inline void stop() { ros_timer_.stop(); }

  inline void callback_ros_(const ros::TimerEvent& event) const { callback_(); };

private:
  ros::Timer ros_timer_;
  std::function<void()> callback_;
};

class ROSTimerInterface : public mavrosflight::TimerInterface
{
public:
  inline std::shared_ptr<mavrosflight::AbstractTimer> createTimer(std::chrono::nanoseconds period,
                                                                  std::function<void()> callback,
                                                                  const bool oneshot = false,
                                                                  const bool autostart = true)
  {
    timer_vec_.push_back(std::make_shared<ROSAbstractTimer>(period, callback, oneshot, autostart));
    return timer_vec_.back();
  }

private:
  std::vector<std::shared_ptr<ROSAbstractTimer>> timer_vec_;
};

} // namespace rosflight
#endif /* ROSFLIGHT_ROS_TIMER_H */