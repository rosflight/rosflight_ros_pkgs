/**
 * \file param.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_PARAM_H
#define MAVROSFLIGHT_PARAM_H

#include <mavrosflight/mavlink_bridge.h>
#include <mavrosflight/mavlink_serial.h>

#include <yaml-cpp/yaml.h>

namespace mavrosflight
{

class Param
{
public:
  Param();
  Param(mavlink_param_value_t msg);
  Param(std::string name, int index, MAV_PARAM_TYPE type, float raw_value);

  uint16_t pack_param_set_msg(uint8_t system, uint8_t component, mavlink_message_t *msg,
                              uint8_t target_system, uint8_t target_component);

  std::string getName() const;
  int getIndex() const;
  MAV_PARAM_TYPE getType() const;
  double getValue() const;

  void setName(std::string name);
  void setIndex(int index);
  void setType(MAV_PARAM_TYPE type);
  void setValue(double value);

  void requestSet(double value, mavlink_message_t *msg);
  bool handleUpdate(const mavlink_param_value_t &msg);

  YAML::Node toYaml();

private:
  void init(std::string name, int index, MAV_PARAM_TYPE type, float raw_value);

  void setFromRawValue(float raw_value);
  float getRawValue();
  float getRawValue(double value);
  double getCastValue(double value);

  template<typename T>
  double fromRawValue(float value)
  {
    T t_value = *(T*) &value;
    return (double) t_value;
  }

  template<typename T>
  float toRawValue(double value)
  {
    T t_value = (T) value;
    return *(float*) &t_value;
  }

  template<typename T>
  double toCastValue(double value)
  {
    return (double) ((T) value);
  }

  MavlinkSerial *serial_;

  std::string name_;
  int index_;
  MAV_PARAM_TYPE type_;
  double value_;

  bool set_in_progress_;
  double new_value_;
  float expected_raw_value_;
};

YAML::Emitter& operator << (YAML::Emitter& out, const Param& param);

} // namespace mavrosflight

//namespace YAML {
//template<>
//struct convert<mavrosflight::Param>
//{
//  static Node encode(const mavrosflight::Param& rhs)
//  {
//    Node node;
//    node["index"] = rhs.getIndex();
//    node["type"] = (int) rhs.getType();
//    node["value"] = rhs.getValue();
//    return node;
//  }

//  static bool decode(const Node& node, mavrosflight::Param& rhs)
//  {
//    if(!node.IsMap() || !node["index"] || !node["type"] || !node["value"])
//    {
//      return false;
//    }

//    rhs.setIndex(node["index"].as<int>());
//    rhs.setType((MAV_PARAM_TYPE) node["type"].as<int>());
//    rhs.setValue(node["value"].as<double>());

//    return true;
//  }
//};
//}

#endif // MAVROSFLIGHT_PARAM_H
