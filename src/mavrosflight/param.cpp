/**
 * \file param.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/param.h>

namespace mavrosflight
{

Param::Param()
{
  init("", -1, MAV_PARAM_TYPE_ENUM_END, 0.0f);
}

Param::Param(mavlink_param_value_t msg)
{
  init(std::string(msg.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN),
       msg.param_index,
       (MAV_PARAM_TYPE) msg.param_type,
       msg.param_value);
}

Param::Param(std::string name, int index, MAV_PARAM_TYPE type, float raw_value)
{
  init(name, index, type, raw_value);
}

uint16_t Param::pack_param_set_msg(uint8_t system, uint8_t component, mavlink_message_t *msg,
                                   uint8_t target_system, uint8_t target_component)
{
  float value;
  switch (type_)
  {
  case MAV_PARAM_TYPE_INT8:
    value = toParamValue<int8_t>(value_);
    break;
  case MAV_PARAM_TYPE_INT16:
    value = toParamValue<int16_t>(value_);
    break;
  case MAV_PARAM_TYPE_INT32:
    value = toParamValue<int32_t>(value_);
    break;
  case MAV_PARAM_TYPE_UINT8:
    value = toParamValue<uint8_t>(value_);
    break;
  case MAV_PARAM_TYPE_UINT16:
    value = toParamValue<uint16_t>(value_);
    break;
  case MAV_PARAM_TYPE_UINT32:
    value = toParamValue<uint32_t>(value_);
    break;
  case MAV_PARAM_TYPE_REAL32:
    value = toParamValue<float>(value_);
    break;
  }

  return mavlink_msg_param_set_pack(system, component, msg,
                                    target_system, target_component, name_.c_str(), value, type_);
}

std::string Param::getName()
{
  return name_;
}

int Param::getIndex()
{
  return index_;
}

MAV_PARAM_TYPE Param::getType()
{
  return type_;
}

double Param::getValue()
{
  return value_;
}

void Param::initializeSet(double value)
{
}

void Param::setValue(double value)
{
  value_ = value;
}

void Param::init(std::string name, int index, MAV_PARAM_TYPE type, float raw_value)
{
  name_ = name;
  index_ = index;
  type_ = type;

  switch (type)
  {
  case MAV_PARAM_TYPE_INT8:
    value_ = fromParamValue<int8_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_INT16:
    value_ = fromParamValue<int16_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_INT32:
    value_ = fromParamValue<int32_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_UINT8:
    value_ = fromParamValue<uint8_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_UINT16:
    value_ = fromParamValue<uint16_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_UINT32:
    value_ = fromParamValue<uint32_t>(raw_value);
    break;
  case MAV_PARAM_TYPE_REAL32:
    value_ = fromParamValue<float>(raw_value);
    break;
  }
}

} // namespace mavrosflight
