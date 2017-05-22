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
 * \file serial_exception.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_SERIAL_EXCEPTION_H
#define MAVROSFLIGHT_SERIAL_EXCEPTION_H

#include <exception>
#include <string>
#include <sstream>

#include <boost/system/system_error.hpp>

namespace mavrosflight
{

/**
 * \brief Describes an exception encountered while using the boost serial libraries
 */
class SerialException : public std::exception
{
public:
  explicit SerialException(const char * const description)
  {
    init(description);
  }

  explicit SerialException(const std::string &description)
  {
    init(description.c_str());
  }

  explicit SerialException(const boost::system::system_error &err)
  {
    init(err.what());
  }

  SerialException(const SerialException &other) : what_(other.what_) {}

  ~SerialException() throw() {}

  virtual const char* what() const throw()
  {
    return what_.c_str();
  }

private:
  std::string what_;

  void init(const char * const description)
  {
    std::ostringstream ss;
    ss << "Serial Error: " << description;
    what_ = ss.str();
  }
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_SERIAL_EXCEPTION_H
