/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
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
 * \file rosflight_io_node.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 * \author Brandon Sutherland <brandonsutherland2@gmail.com>
 */

#include <rclcpp/rclcpp.hpp>
#include <rosflight_io/rosflight_io.hpp>

#include <pthread.h>
#include <sys/types.h>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>

#include <string>
#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class RealtimeConfigurator
{
public:
  explicit RealtimeConfigurator(int who = RUSAGE_THREAD)
  : who_{who} {}

  void init() noexcept
  {
    involuntary_context_switches_previous_ = get_involuntary_context_switches(who_);
  }

  [[nodiscard]] int64_t num_context_switches() noexcept
  {
    std::call_once(
      once_, [this] {init();}
    );
    int64_t current = get_involuntary_context_switches(who_);
    return current - std::exchange(involuntary_context_switches_previous_, current);
  }
  
  int64_t get_involuntary_context_switches(int who = RUSAGE_THREAD) noexcept
  {
    struct rusage rusage {};
    getrusage(who, &rusage);
    return static_cast<int64_t>(rusage.ru_nivcsw);
  }
  
  void configure_thread_for_realtime(int priority = 85, int policy = SCHED_RR)
  {
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // The middleware threads are created on the node construction.
    // Since the main thread settings are configured before the middleware threads
    // are created, the middleware threads will inherit the scheduling settings.
    // From: https://linux.die.net/man/3/pthread_create
    // "The new thread inherits copies of the calling thread's capability sets
    // (see capabilities(7)) and CPU affinity mask (see sched_setaffinity(2))."

    set_thread_scheduling(pthread_self(), policy, priority);
  }

  void configure_node_to_report_context_switches(rclcpp::Node::SharedPtr& node)
  {
    publisher_ = 
      node->create_publisher<std_msgs::msg::Int32>("rosflight_io/context_switches", 10);

    context_timer_ = node->create_wall_timer(1000ms, std::bind(&RealtimeConfigurator::publish_context_switches, this));
  }

private:
  int64_t involuntary_context_switches_previous_;
  const int who_;
  std::once_flag once_;
  rclcpp::TimerBase::SharedPtr context_timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  void publish_context_switches()
  {
    auto context_switches = num_context_switches();
    std_msgs::msg::Int32 msg;
    msg.data = context_switches;
    publisher_->publish(msg);
  }

  void set_thread_scheduling(std::thread::native_handle_type thread, int policy, int sched_priority)
  {
    struct sched_param param;
    param.sched_priority = sched_priority;
    auto ret = pthread_setschedparam(thread, policy, &param);
    if (ret > 0) {
      throw std::runtime_error("Couldn't set scheduling priority and policy. Error code: " + std::string(strerror(errno)));
    }
  }
};


int main(int argc, char ** argv)
{
  bool is_realtime = false; // TODO: Make these arguments
  bool is_publish_context_switches = true;
  int priority = 87;
  int policy = SCHED_RR;

  RealtimeConfigurator configure;

  if (is_realtime) {
    configure.configure_thread_for_realtime(policy, priority);
  }

  rclcpp::init(argc, argv);

  rosflight_io::ROSflightIO::SharedPtr node = std::make_shared<rosflight_io::ROSflightIO>();

  if (is_publish_context_switches) {
    configure.configure_node_to_report_context_switches(node);
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
