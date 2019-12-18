#include <rosflight/mavrosflight/config_manager.h>
#include <iostream> //testing

constexpr std::chrono::milliseconds mavrosflight::ConfigManager::timeout; //Why, C++?
mavrosflight::ConfigManager::ConfigManager(mavrosflight::MavlinkComm *const comm) : comm_{comm}
{
  comm_->register_mavlink_listener(this);
}

mavrosflight::ConfigManager::~ConfigManager()
{
  for (config_promise_t *promise:promises_)
    delete promise;
}

void mavrosflight::ConfigManager::handle_mavlink_message(const mavlink_message_t &msg)
{
  switch (msg.msgid)
  {
    case MAVLINK_MSG_ID_ROSFLIGHT_CONFIG:
      handle_config_message(msg);
      break;
    case MAVLINK_MSG_ID_ROSFLIGHT_DEVICE_INFO:
      handle_device_info_message(msg);
      break;
    case MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_INFO:
      handle_config_info_message(msg);
      break;
    default:
      if(msg.msgid >= 200)
        std::cout<<static_cast<int>(msg.msgid)<<std::endl;
  }
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

bool mavrosflight::ConfigManager::set_configuration(uint8_t device, uint8_t config)
{
  send_config(device, config);
  // In the future, there may be a response, which would be handled here
  return true;
}

void mavrosflight::ConfigManager::request_config_info()
{
  send_config_request(0xff);
}

void mavrosflight::ConfigManager::send_config(uint8_t device, uint8_t config)
{
  mavlink_message_t config_message;
  mavlink_msg_rosflight_config_pack(1, 0, &config_message, device, config);
  comm_->send_message(config_message);
}

void mavrosflight::ConfigManager::handle_device_info_message(const mavlink_message_t &msg)
{
  mavlink_rosflight_device_info_t device_info;
  mavlink_msg_rosflight_device_info_decode(&msg, &device_info);
  uint8_t device = device_info.device;
  if(device >= device_names_.size())
    device_names_.resize(device+1);
  device_names_[device].name = std::string{reinterpret_cast<char*>(device_info.name)};
  device_names_[device].max_value = device_info.max_value;
  std::cout<<"Device info recieved: "<<static_cast<int>(device)<<','<< device_names_[device].name<<std::endl;
}

void mavrosflight::ConfigManager::handle_config_info_message(const mavlink_message_t &msg)
{
  mavlink_rosflight_config_info_t config_info;
  mavlink_msg_rosflight_config_info_decode(&msg, &config_info);
  uint8_t device = config_info.device;
  uint8_t config = config_info.config;
  if(device >= device_names_.size())
    device_names_.resize(device+1);
  if(config >= device_names_[device].config_names.size())
    device_names_[device].config_names.resize(config+1);
  device_names_[device].config_names[config] = std::string{reinterpret_cast<char*>(config_info.name)};
  std::cout<<"Config info recieved: "<<static_cast<int>(device)<<','<<static_cast<int>(config)<<','<<device_names_[device].config_names[config]<<std::endl;
}
