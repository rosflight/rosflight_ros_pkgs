#include <rosflight/mavrosflight/config_manager.h>

constexpr std::chrono::milliseconds mavrosflight::ConfigManager::timeout; //Why, C++?
mavrosflight::ConfigManager::ConfigManager(mavrosflight::MavlinkComm *const comm) : comm_{comm}
{

}

mavrosflight::ConfigManager::~ConfigManager()
{
  for (config_promise_t *promise:promises_)
    delete promise;
}

void mavrosflight::ConfigManager::handle_mavlink_message(const mavlink_message_t &msg)
{
  if (msg.msgid == MAVLINK_MSG_ID_ROSFLIGHT_CONFIG)
    handle_config_message(msg);
}

void mavrosflight::ConfigManager::handle_config_message(const mavlink_message_t &msg)
{
  mavlink_rosflight_config_t config_msg;
  mavlink_msg_rosflight_config_decode(&msg, &config_msg);
  uint8_t device = config_msg.device;
  uint8_t config = config_msg.config;
  for (size_t promise_index{0}; promise_index < promises_.size(); promise_index++)
    if (promises_[promise_index]->device == device)
    {
      promises_[promise_index]->promise.set_value(config);
      promises_.erase(promises_.begin() + promise_index);
      break;
    }
}

bool mavrosflight::ConfigManager::get_configuration(uint8_t device, uint8_t &config)
{
  config_promise_t *promise = new config_promise_t;
  promise->device = device;
  promises_.push_back(promise);

  std::future<uint8_t> future = promise->promise.get_future();
  send_config_request(device);
  std::future_status result = future.wait_for(timeout);
  bool success;
  if (result == std::future_status::ready)
  {
    config = future.get();
    success = true;
  } else
    success = false;
  delete promise;
  return success;
}

void mavrosflight::ConfigManager::send_config_request(uint8_t device)
{
  mavlink_message_t config_request_message;
  mavlink_msg_rosflight_config_request_pack(1, 0, &config_request_message, device);
  comm_->send_message(config_request_message);
}

void mavrosflight::ConfigManager::set_configuration(uint8_t device, uint8_t config)
{
  send_config(device, config); // In the future, there may be a response, which would be handled here
}

void mavrosflight::ConfigManager::send_config(uint8_t device, uint8_t config)
{
  mavlink_message_t config_message;
  mavlink_msg_rosflight_config_pack(1, 0, &config_message, device, config);
  comm_->send_message(config_message);
}
