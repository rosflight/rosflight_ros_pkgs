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
 * \file param.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_PARAM_H
#define MAVROSFLIGHT_PARAM_H

#include <rosflight/mavrosflight/mavlink_bridge.hpp>
#include <rosflight/mavrosflight/mavlink_serial.hpp>

#include <yaml-cpp/yaml.h>

namespace mavrosflight
{
class Param
{
public:
  Param();
  explicit Param(mavlink_param_value_t msg);
  Param(std::string name, int index, MAV_PARAM_TYPE type, float raw_value);

  std::string getName() const;
  int getIndex() const;
  MAV_PARAM_TYPE getType() const;
  double getValue() const;

  void requestSet(double value, mavlink_message_t * msg);
  bool handleUpdate(const mavlink_param_value_t & msg);

private:
  void init(std::string name, int index, MAV_PARAM_TYPE type, float raw_value);

  void setFromRawValue(float raw_value);
  float getRawValue();
  float getRawValue(double value);
  double getCastValue(double value);

  template<typename T>
  double fromRawValue(float value)
  {
    T t_value;
    std::memcpy(&t_value, &value, sizeof(T));
    return static_cast<double>(t_value);
  }

  template<typename T>
  float toRawValue(double value)
  {
    T t_value = static_cast<T>(value);
    float result = 0.0f;
    std::memcpy(&result, &t_value, sizeof(T));
    return result;
  }

  template<typename T>
  double toCastValue(double value)
  {
    return static_cast<double>(static_cast<T>(value));
  }

  std::string name_;
  int index_;
  MAV_PARAM_TYPE type_;
  double value_;

  bool set_in_progress_;
  double new_value_;
  float expected_raw_value_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_PARAM_H
