#include "mavrosflight/param_manager.h"

#include <stdint.h>

namespace mavrosflight
{

ParamManager::ParamManager(MavlinkSerial * const serial) :
  serial_(serial),
  unsaved_changes_(false),
  write_request_in_progress_(false),
  first_param_received_(false),
  param_count_(0),
  initialized_(false)
{
  serial_->register_mavlink_listener(this);
}

ParamManager::~ParamManager()
{
  if (param_count_ > 0)
  {
    delete[] received_;
  }
}

void ParamManager::handle_mavlink_message(const mavlink_message_t &msg)
{
  switch (msg.msgid)
  {
  case MAVLINK_MSG_ID_PARAM_VALUE:
    handle_param_value_msg(msg);
    break;
  case MAVLINK_MSG_ID_COMMAND_ACK:
    handle_command_ack_msg(msg);
    break;
  }
}

bool ParamManager::is_param_id(std::string name)
{
  return (params_.find(name) != params_.end());
}

bool ParamManager::unsaved_changes()
{
  return unsaved_changes_;
}

void ParamManager::handle_param_value_msg(const mavlink_message_t &msg)
{
  mavlink_param_value_t param;
  mavlink_msg_param_value_decode(&msg, &param);

  if (!first_param_received_)
  {
    first_param_received_ = true;
    received_ = new bool[param.param_count];
  }
  received_[param.param_index] = true; //! \todo Implement check that all parameters have been received

  std::string name(param.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
  params_[name] = Param(param);
}

void ParamManager::handle_command_ack_msg(const mavlink_message_t &msg)
{
  if (write_request_in_progress_)
  {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);

    if (ack.command == MAV_CMD_PREFLIGHT_STORAGE && ack.result == MAV_CMD_ACK_OK)
    {
      write_request_in_progress_ = false;
      unsaved_changes_ = false;
    }
  }
}

bool ParamManager::set_param_value(std::string name, double value)
{
  if (!is_param_id(name))
    return false;

  Param *param = &params_[name];
  bool value_changed = (value != param->getValue());
  param->setValue(value);

  if (value_changed)
  {
    mavlink_message_t msg;
    param->pack_param_set_msg(1, 1, &msg, 1, 1);
    serial_->send_message(msg);

    unsaved_changes_ = true;
  }

  return true;
}

void ParamManager::write_params()
{
  if (!write_request_in_progress_)
  {
    mavlink_message_t msg;
    uint8_t sysid = 1;
    uint8_t compid = 1;
    mavlink_msg_command_int_pack(sysid, compid, &msg,
                                 1, MAV_COMP_ID_ALL, 0, MAV_CMD_PREFLIGHT_STORAGE, 0, 0, 1, 0, 0, 0, 0, 0, 0);
    serial.send_message(msg);

    write_request_in_progress_ = true;
  }
}

} // namespace mavrosflight
