/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2018 Daniel Koch.
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
 * @author Daniel Koch <danielpkoch@gmail.com>
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
  template <typename... T>
  inline void debug(const std::string &format, const T &... args)
  {
    ROS_DEBUG(format.c_str(), args...);
  }
  template <typename... T>
  inline void debug_throttle(float period, const std::string &format, const T &... args)
  {
    ROS_DEBUG_THROTTLE(period, format.c_str(), args...);
  }

  template <typename... T>
  inline void info(const std::string &format, const T &... args)
  {
    ROS_INFO(format.c_str(), args...);
  }
  template <typename... T>
  inline void info_throttle(float period, const std::string &format, const T &... args)
  {
    ROS_INFO_THROTTLE(period, format.c_str(), args...);
  }

  template <typename... T>
  inline void warn(const std::string &format, const T &... args)
  {
    ROS_WARN(format.c_str(), args...);
  }
  template <typename... T>
  inline void warn_throttle(float period, const std::string &format, const T &... args)
  {
    ROS_WARN_THROTTLE(period, format.c_str(), args...);
  }

  template <typename... T>
  inline void error(const std::string &format, const T &... args)
  {
    ROS_ERROR(format.c_str(), args...);
  }
  template <typename... T>
  inline void error_throttle(float period, const std::string &format, const T &... args)
  {
    ROS_ERROR_THROTTLE(period, format.c_str(), args...);
  }

  template <typename... T>
  inline void fatal(const std::string &format, const T &... args)
  {
    ROS_FATAL(format.c_str(), args...);
  }
  template <typename... T>
  inline void fatal_throttle(float period, const std::string &format, const T &... args)
  {
    ROS_FATAL_THROTTLE(period, format.c_str(), args...);
  }
};

} // namespace rosflight

#endif // ROSFLIGHT_ROS_LOGGER_H