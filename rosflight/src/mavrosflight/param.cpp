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
 * \file param.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <rosflight/mavrosflight/param.h>

namespace mavrosflight
{

Param::Param()
{
  init("", -1, MAV_PARAM_TYPE_ENUM_END, 0.0f);
}

Param::Param(mavlink_param_value_t msg)
{
  char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
  memcpy(name, msg.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
  name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

  init(std::string(name),
       msg.param_index,
       (MAV_PARAM_TYPE) msg.param_type,
       msg.param_value);
}

Param::Param(std::string name, int index, MAV_PARAM_TYPE type, float raw_value)
{
  init(name, index, type, raw_value);
}

std::string Param::getName() const
{
  return name_;
}

int Param::getIndex() const
{
  return index_;
}

MAV_PARAM_TYPE Param::getType() const
{
  return type_;
}

double Param::getValue() const
{
  return value_;
}

void Param::requestSet(double value, mavlink_message_t *msg)
{
  if (value != value_)
  {
    new_value_ = getCastValue(value);
    expected_raw_value_ = getRawValue(new_value_);

    mavlink_msg_param_set_pack(1, 50, msg,
                               1, MAV_COMP_ID_ALL, name_.c_str(), expected_raw_value_, type_);

    set_in_progress_ = true;
  }
}

bool Param::handleUpdate(const mavlink_param_value_t &msg)
{
  if (msg.param_index != index_)
    return false;

  if (msg.param_type != type_)
    return false;

  if (set_in_progress_ && msg.param_value == expected_raw_value_)
    set_in_progress_ = false;

  if (msg.param_value != getRawValue())
  {
    setFromRawValue(msg.param_value);
    return true;
  }

  return false;
}

void Param::init(std::string name, int index, MAV_PARAM_TYPE type, float raw_value)
{
  name_ = name;
  index_ = index;
  type_ = type;
  setFromRawValue(raw_value);
  set_in_progress_ = false;
}

void Param::setFromRawValue(float raw_value)
{
  switch (type_)
  {
  case MAV_PARAM_TYPE_INT8:
    value_ = fromRawValue<int8_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_INT16:
    value_ = fromRawValue<int16_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_INT32:
    value_ = fromRawValue<int32_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_UINT8:
    value_ = fromRawValue<uint8_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_UINT16:
    value_ = fromRawValue<uint16_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_UINT32:
    value_ = fromRawValue<uint32_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_REAL32:
    value_ = fromRawValue<float>(raw_value);
    break;
  }
}

float Param::getRawValue()
{
  return getRawValue(value_);
}

float Param::getRawValue(double value)
{
  float raw_value;

  switch (type_)
  {
  case MAV_PARAM_TYPE_INT8:
    raw_value = toRawValue<int8_t>(value);
    break;
  case MAV_PARAM_TYPE_INT16:
    raw_value = toRawValue<int16_t>(value);
    break;
  case MAV_PARAM_TYPE_INT32:
    raw_value = toRawValue<int32_t>(value);
    break;
  case MAV_PARAM_TYPE_UINT8:
    raw_value = toRawValue<uint8_t>(value);
    break;
  case MAV_PARAM_TYPE_UINT16:
    raw_value = toRawValue<uint16_t>(value);
    break;
  case MAV_PARAM_TYPE_UINT32:
    raw_value = toRawValue<uint32_t>(value);
    break;
  case MAV_PARAM_TYPE_REAL32:
    raw_value = toRawValue<float>(value);
    break;
  }

  return raw_value;
}

double Param::getCastValue(double value)
{
  double cast_value;

  switch (type_)
  {
  case MAV_PARAM_TYPE_INT8:
    cast_value = toCastValue<int8_t>(value);
    break;
  case MAV_PARAM_TYPE_INT16:
    cast_value = toCastValue<int16_t>(value);
    break;
  case MAV_PARAM_TYPE_INT32:
    cast_value = toCastValue<int32_t>(value);
    break;
  case MAV_PARAM_TYPE_UINT8:
    cast_value = toCastValue<uint8_t>(value);
    break;
  case MAV_PARAM_TYPE_UINT16:
    cast_value = toCastValue<uint16_t>(value);
    break;
  case MAV_PARAM_TYPE_UINT32:
    cast_value = toCastValue<uint32_t>(value);
    break;
  case MAV_PARAM_TYPE_REAL32:
    cast_value = toCastValue<float>(value);
    break;
  }

  return cast_value;
}

} // namespace mavrosflight
