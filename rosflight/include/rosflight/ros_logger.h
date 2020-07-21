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
 * @file ros_logger.h
 * @author Jacob Willis <jbwillis272@gmail.com>
 */

#ifndef ROSFLIGHT_ROS_LOGGER_H
#define ROSFLIGHT_ROS_LOGGER_H

#include <rosflight/mavrosflight/logger_interface.h>

#include <ros/ros.h>

namespace rosflight
{
/**
 * @class ROSLogger
 * @brief Logger implementation for ROS environments
 *
 * This is a convenience logger implementation for ROS-based projects.
 * The implementation simply forwards messages to the appropriate rosconsole loggers.
 */
class ROSLogger : public mavrosflight::LoggerInterface<ROSLogger>
{
public:
  template <typename... Args>
  inline void debug(const char* format, const Args&... args)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    ROS_DEBUG(format, args...);
#pragma GCC diagnostic pop
  }
  template <typename... Args>
  inline void debug_throttle(float period, const char* format, const Args&... args)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    ROS_DEBUG_THROTTLE(period, format, args...);
#pragma GCC diagnostic pop
  }

  template <typename... Args>
  inline void info(const char* format, const Args&... args)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    ROS_INFO(format, args...);
#pragma GCC diagnostic pop
  }
  template <typename... Args>
  inline void info_throttle(float period, const char* format, const Args&... args)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    ROS_INFO_THROTTLE(period, format, args...);
#pragma GCC diagnostic pop
  }

  template <typename... Args>
  inline void warn(const char* format, const Args&... args)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    ROS_WARN(format, args...);
#pragma GCC diagnostic pop
  }
  template <typename... Args>
  inline void warn_throttle(float period, const char* format, const Args&... args)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    ROS_WARN_THROTTLE(period, format, args...);
#pragma GCC diagnostic pop
  }

  template <typename... Args>
  inline void error(const char* format, const Args&... args)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    ROS_ERROR(format, args...);
#pragma GCC diagnostic pop
  }
  template <typename... Args>
  inline void error_throttle(float period, const char* format, const Args&... args)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    ROS_ERROR_THROTTLE(period, format, args...);
#pragma GCC diagnostic pop
  }

  template <typename... Args>
  inline void fatal(const char* format, const Args&... args)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    ROS_FATAL(format, args...);
#pragma GCC diagnostic pop
  }
  template <typename... Args>
  inline void fatal_throttle(float period, const char* format, const Args&... args)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    ROS_FATAL_THROTTLE(period, format, args...);
#pragma GCC diagnostic pop
  }
};

} // namespace rosflight

#endif // ROSFLIGHT_ROS_LOGGER_H