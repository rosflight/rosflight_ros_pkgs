/**
 * \file param.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_PARAM_H
#define MAVROSFLIGHT_PARAM_H

#include <mavrosflight/mavlink_bridge.h>

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

  std::string getName();
  int getIndex();
  MAV_PARAM_TYPE getType();
  double getValue();
  bool getSetInProgress();

  void initializeSet(double value);
  bool confirmSet();

private:
  void init(std::string name, int index, MAV_PARAM_TYPE type, float raw_value);

  template<typename T>
  double fromParamValue(float value)
  {
    T t_value = *(T*) &value;
    return (double) t_value;
  }

  template<typename T>
  float toParamValue(double value)
  {
    T t_value = (T) value;
    return *(float*) &t_value;
  }

  std::string name_;
  int index_;
  MAV_PARAM_TYPE type_;
  double value_;

  bool set_in_progress_;
  float requested_value_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_PARAM_H
