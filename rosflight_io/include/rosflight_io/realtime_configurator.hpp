/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2026 Ian Reid, BYU MAGICC Lab.
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
 * \file realtime_configurator.hpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 * \author Brandon Sutherland <brandonsutherland2@gmail.com>
 */
#ifndef ROSFLIGHT_IO_REALTIME_CONFIGURATOR_HPP
#define ROSFLIGHT_IO_REALTIME_CONFIGURATOR_HPP

#include <cstdint>
#include <mutex>
#include <thread>
#include <utility>

#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <sched.h>
#include <std_msgs/msg/int32.hpp>
#include <sys/resource.h>
#include <algorithm>
#include <cctype>
#include <string>

class RealtimeConfigurator
{
public:
  explicit RealtimeConfigurator(int who = RUSAGE_THREAD);

  void configure(int argc, char ** argv);

  void init() noexcept;

  [[nodiscard]] int64_t num_context_switches() noexcept;

  int64_t get_involuntary_context_switches(int who = RUSAGE_THREAD) noexcept;

  void configure_thread_for_realtime();

  void configure_node_to_report_context_switches(rclcpp::Node::SharedPtr& node);

  bool is_realtime();
  
  bool is_publish_context_switches();

private:
  int64_t involuntary_context_switches_previous_;
  const int who_;
  std::once_flag once_;
  rclcpp::TimerBase::SharedPtr context_timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  int priority_;
  int policy_;
  bool is_realtime_;
  bool is_publish_context_switches_;

  void publish_context_switches();

  void set_thread_scheduling(std::thread::native_handle_type thread, int policy, int sched_priority);
};

#endif  // ROSFLIGHT_IO_REALTIME_CONFIGURATOR_HPP
