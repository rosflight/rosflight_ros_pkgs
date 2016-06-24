/**
 * \file mavrosflight.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Sources:
 * https://gist.github.com/yoggy/3323808
 */

#include "mavrosflight/mavrosflight.h"

#include <ros/ros.h>

namespace mavrosflight
{

using boost::asio::serial_port_base;

MavROSflight::MavROSflight(std::string port, int baud_rate, uint8_t sysid /* = 1 */, uint8_t compid /* = 50 */) :
  serial(port, baud_rate),
  param(&serial),
  sysid_(sysid),
  compid_(compid)
{}

MavROSflight::~MavROSflight()
{}

void MavROSflight::send_param_request_list(uint8_t target_system, uint8_t target_component)
{
  mavlink_message_t msg;
  mavlink_msg_param_request_list_pack(sysid_, compid_, &msg, target_system, target_component);
  serial.send_message(msg);
}

void MavROSflight::send_param_request_read(uint8_t target_system, uint8_t target_component, std::string name)
{
  mavlink_message_t msg;
  mavlink_msg_param_request_read_pack(sysid_, compid_, &msg, target_system, target_component, name.c_str(), -1);
  serial.send_message(msg);
}

void MavROSflight::send_param_set(uint8_t target_system, uint8_t target_component, std::string param_id, int32_t param_value)
{
  mavlink_message_t msg;
  mavlink_msg_param_set_pack(sysid_, compid_, &msg,
                             target_system, target_component, param_id.c_str(), *(float*) &param_value, MAV_PARAM_TYPE_INT32);
  serial.send_message(msg);
}

void MavROSflight::send_param_write(uint8_t target_system, uint8_t target_component)
{
  mavlink_message_t msg;
  mavlink_msg_command_int_pack(sysid_, compid_, &msg,
                               target_system, target_component, 0, MAV_CMD_PREFLIGHT_STORAGE, 0, 0, 1, 0, 0, 0, 0, 0, 0);
  serial.send_message(msg);
}

void MavROSflight::send_command(OFFBOARD_CONTROL_MODE mode, OFFBOARD_CONTROL_IGNORE ignore,
                                float value1, float value2, float value3, float value4)
{
  int v1 = (int) (value1 * 1000);
  int v2 = (int) (value2 * 1000);
  int v3 = (int) (value3 * 1000);
  int v4 = (int) (value4 * 1000);

  switch (mode)
  {
  case MODE_PASS_THROUGH:
    v1 = saturate(v1, -1000, 1000);
    v2 = saturate(v2, -1000, 1000);
    v3 = saturate(v3, -1000, 1000);
    v4 = saturate(v4, -1000, 1000);
    break;
  case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
  case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
    v4 = saturate(v4, 0, 1000);
    break;
  case MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
    break;
  }

  mavlink_message_t msg;
  mavlink_msg_offboard_control_pack(sysid_, compid_, &msg, mode, ignore,
                                    (int16_t)v1, (int16_t)v2, (int16_t)v3, (int16_t)v4);
  serial.send_message(msg);
}

} // namespace mavrosflight
