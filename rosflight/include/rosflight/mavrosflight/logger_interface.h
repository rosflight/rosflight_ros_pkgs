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
 * @file logger_interface.h
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#ifndef MAVROSFLIGHT_LOGGER_INTERFACE_H
#define MAVROSFLIGHT_LOGGER_INTERFACE_H

#include <iostream>
#include <string>

namespace mavrosflight
{

/**
 * \class LoggerInterface
 * \brief Abstract base class for message handler
 *
 * The implementations of this class define how messages are displayed, logged,
 * etc. To create custom behavior, derive from this base class and override the
 * pure virtual functions.
 */
class LoggerInterface
{
public:
  virtual void debug(const std::string& message) = 0;
  virtual void debug_throttle(float rate, const std::string& message) = 0;

  virtual void info(const std::string& message) = 0;
  virtual void info_throttle(float rate, const std::string& message) = 0;

  virtual void warn(const std::string& message) = 0;
  virtual void warn_throttle(float rate, const std::string& message) = 0;

  virtual void error(const std::string& message) = 0;
  virtual void error_throttle(float rate, const std::string& message) = 0;

  virtual void fatal(const std::string& message) = 0;
  virtual void fatal_throttle(float rate, const std::string& message) = 0;
};

/**
 * \class DefaultLogger
 * \brief Default logger that outputs to stdout and stderr
 */
class DefaultLogger : public LoggerInterface
{
public:
  inline void debug(const std::string &message) override { std::cout << "[mavrosflight][DEBUG]: " << message << std::endl; }
  inline void debug_throttle(float rate, const std::string &message) override { debug(message); }

  inline void info(const std::string &message) override { std::cout << "[mavrosflight][INFO]: " << message << std::endl; }
  inline void info_throttle(float rate, const std::string &message) override { info(message); }

  inline void warn(const std::string &message) override { std::cerr << "[mavrosflight][WARN]: " << message << std::endl; }
  inline void warn_throttle(float rate, const std::string &message) override { warn(message); }

  inline void error(const std::string &message) override { std::cerr << "[mavrosflight][ERROR]: " << message << std::endl; }
  inline void error_throttle(float rate, const std::string &message) override { error(message); }

  inline void fatal(const std::string &message) override { std::cerr << "[mavrosflight][FATAL]: " << message << std::endl; }
  inline void fatal_throttle(float rate, const std::string &message) override { fatal(message); }
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_LOGGER_INTERFACE_H