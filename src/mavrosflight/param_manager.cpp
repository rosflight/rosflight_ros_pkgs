/**
 * \file param_manager.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/param_manager.h>

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

bool ParamManager::unsaved_changes()
{
  return unsaved_changes_;
}

bool ParamManager::get_param_value(std::string name, double *value)
{
  if (is_param_id(name))
  {
    *value = params_[name].getValue();
    return true;
  }
  else
  {
    *value = 0.0;
    return false;
  }
}

bool ParamManager::set_param_value(std::string name, double value)
{
  if (is_param_id(name))
  {
    mavlink_message_t msg;
    params_[name].requestSet(value, &msg);
    serial_->send_message(msg);

    return true;
  }
  else
  {
    return false;
  }
}

bool ParamManager::write_params()
{
  if (!write_request_in_progress_)
  {
    mavlink_message_t msg;
    uint8_t sysid = 1;
    uint8_t compid = 1;
    mavlink_msg_command_int_pack(sysid, compid, &msg,
                                 1, MAV_COMP_ID_ALL, 0, MAV_CMD_PREFLIGHT_STORAGE, 0, 0, 1, 0, 0, 0, 0, 0, 0);
    serial_->send_message(msg);

    write_request_in_progress_ = true;
    return true;
  }
  else
  {
    return false;
  }
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

  // ensure null termination of name
  char c_name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
  memcpy(c_name, param.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
  c_name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

  std::string name(c_name);

  if (!is_param_id(name)) // if we haven't received this param before, add it
  {
    params_[name] = Param(param);
    received_[param.param_index] = true; //! \todo Implement check that all parameters have been received
  }
  else // otherwise check if we have new unsaved changes as a result of a param set request
  {
    if (params_[name].handleUpdate(param))
      unsaved_changes_ = true;
  }
}

void ParamManager::handle_command_ack_msg(const mavlink_message_t &msg)
{
  if (write_request_in_progress_)
  {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);

    if (ack.command == MAV_CMD_PREFLIGHT_STORAGE && ack.result == MAV_RESULT_ACCEPTED)
    {
      write_request_in_progress_ = false;
      unsaved_changes_ = false;
    }
  }
}

bool ParamManager::is_param_id(std::string name)
{
  return (params_.find(name) != params_.end());
}

} // namespace mavrosflight
