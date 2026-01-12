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
 * \file realtime_configurator.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 * \author Brandon Sutherland <brandonsutherland2@gmail.com>
 */

#include <rosflight_io/realtime_configurator.hpp>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <stdexcept>

using namespace std::chrono_literals;

RealtimeConfigurator::RealtimeConfigurator(int who)
: who_{who} 
{
  // Default settings.
  priority_ = 87;
  policy_ = SCHED_RR;
  is_realtime_ = false;
  is_publish_context_switches_ = false;
}

void RealtimeConfigurator::init() noexcept
{
  involuntary_context_switches_previous_ = get_involuntary_context_switches(who_);
}

[[nodiscard]] int64_t RealtimeConfigurator::num_context_switches() noexcept
{
  std::call_once(
    once_, [this] {init();}
  );
  int64_t current = get_involuntary_context_switches(who_);
  return current - std::exchange(involuntary_context_switches_previous_, current);
}

int64_t RealtimeConfigurator::get_involuntary_context_switches(int who) noexcept
{
  struct rusage rusage {};
  getrusage(who, &rusage);
  return static_cast<int64_t>(rusage.ru_nivcsw);
}

void RealtimeConfigurator::configure_thread_for_realtime()
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // The middleware threads are created on the node construction.
  // Since the main thread settings are configured before the middleware threads
  // are created, the middleware threads will inherit the scheduling settings.
  // From: https://linux.die.net/man/3/pthread_create
  // "The new thread inherits copies of the calling thread's capability sets
  // (see capabilities(7)) and CPU affinity mask (see sched_setaffinity(2))."

  set_thread_scheduling(pthread_self(), policy_, priority_);
}

void RealtimeConfigurator::configure_node_to_report_context_switches(rclcpp::Node::SharedPtr& node)
{
  std::string node_name = std::string(node->get_name());
  publisher_ =
    node->create_publisher<std_msgs::msg::Int32>(node_name + "/context_switches", 10);

  context_timer_ = node->create_wall_timer(1000ms, std::bind(&RealtimeConfigurator::publish_context_switches, this));
}

void RealtimeConfigurator::publish_context_switches()
{
  auto context_switches = num_context_switches();
  std_msgs::msg::Int32 msg;
  msg.data = context_switches;
  publisher_->publish(msg);
}

void RealtimeConfigurator::set_thread_scheduling(std::thread::native_handle_type thread, int policy, int sched_priority)
{
  struct sched_param param;
  param.sched_priority = sched_priority;
  auto ret = pthread_setschedparam(thread, policy, &param);
  if (ret > 0) {
    throw std::runtime_error(
      "Couldn't set scheduling priority and policy. Error code: " + std::string(strerror(errno)));
  }
}

bool RealtimeConfigurator::is_publish_context_switches() {return is_publish_context_switches_;}

bool RealtimeConfigurator::is_realtime() {return is_realtime_;}

void RealtimeConfigurator::configure(int argc, char ** argv) {
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    auto pos = arg.find(":=");
    if (pos == std::string::npos) {
      continue;
    }
    std::string key = arg.substr(0, pos);
    std::string value = arg.substr(pos + 2);
    if (key == "realtime") {
      if (value == "true") {
        is_realtime_ = true;
      }
    } else if (key == "publish_context_switches") {
      if (value == "true") {
        is_publish_context_switches_ = true;
      }
    } else if (key == "priority") {
      try {
        priority_ = std::stoi(value);
      } catch (const std::exception&) {
        std::cout << "Priority must be an integer.\n";
      }
    } else if (key == "policy") {
      if (value == "RR") {
        policy_ = SCHED_RR;
      } else if (value == "FIFO") {
        policy_ = SCHED_FIFO;
      }
      else {
        std::cout << "Policy must be RR or FIFO\n";
      }
    }
  }
}

