#include <rosflight/mavrosflight/config_manager.h>

#include <boost/algorithm/string.hpp>
#include <ros/ros.h>


namespace mavrosflight
{
constexpr std::chrono::milliseconds ConfigManager::TIMEOUT; //Why, C++?
ConfigManager::ConfigManager(MavlinkComm *const comm) : comm_{comm}
{
  comm_->register_mavlink_listener(this);
}

ConfigManager::~ConfigManager()
{
  for (ConfigPromise *promise:promises_)
    delete promise;
  for(ConfigResponsePromise *response_promise: config_response_promises_)
    delete response_promise;
}

void ConfigManager::handle_mavlink_message(const mavlink_message_t &msg)
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
  case MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS:
    handle_config_response_message(msg);
    break;
  default:
    break;
  }
}

void ConfigManager::handle_config_message(const mavlink_message_t &msg)
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

std::tuple<bool, uint8_t> ConfigManager::get_configuration(uint8_t device)
{
  ConfigPromise *promise = new ConfigPromise;
  promise->device = device;
  promises_.push_back(promise);

  std::future<uint8_t> future = promise->promise.get_future();
  send_config_get_request(device);
  std::future_status result = future.wait_for(TIMEOUT);
  bool success;
  uint8_t config{0};
  if (result == std::future_status::ready)
  {
    config = future.get();
    success = true;
  } else
  {
    success = false;
    for(size_t promise_index{0}; promise_index < promises_.size(); promise_index++)
      if(promises_[promise_index]==promise)
      {
        promises_.erase(promises_.begin() + promise_index);
        break;
      }
  }
  delete promise;
  return std::make_tuple(success, config);
}

void ConfigManager::send_config_get_request(uint8_t device)
{
  mavlink_message_t config_request_message;
  mavlink_msg_rosflight_config_request_pack(1, 0, &config_request_message, device);
  comm_->send_message(config_request_message);
}

void ConfigManager::restart_config_info_request()
{
  ROS_WARN_THROTTLE(5,"Error getting configuration info. Restarting.");
  device_info_.clear();
  request_config_info();
}
void ConfigManager::restart_config_info_request(const ros::TimerEvent &event)
{
  restart_config_info_request();
}
void ConfigManager::restart_config_receive_timer()
{
  if (config_receive_timer_.hasStarted())
  {
    config_receive_timer_.stop();
    config_receive_timer_.start();
  }
}
void ConfigManager::finish_config_info_receive()
{
  config_receive_timer_.stop();
  ROS_INFO("Received all configuration info.");
}

ConfigManager::ConfigResponse
ConfigManager::set_configuration(uint8_t device, uint8_t config)
{
  ConfigResponsePromise *promise = new ConfigResponsePromise;
  promise->device = device;
  promise->config = config;
  std::future<ConfigResponse> future = promise->promise.get_future();
  config_response_promises_.push_back(promise);
  send_config_set_request(device, config);
  std::future_status result = future.wait_for(TIMEOUT);

  ConfigResponse response;

  if (result == std::future_status::ready) // A response was received
  {
    response = future.get();
  } else // no response was received
  {
    response.successful = false;
    response.reboot_required = false;
    response.error_message = "No response received. The configuration may or may not be set";
    for (size_t promise_index{0}; promise_index < config_response_promises_.size(); promise_index++)
      if (config_response_promises_[promise_index] == promise)
      {
        config_response_promises_.erase(config_response_promises_.begin() + promise_index);
        break;
      }
  }
  delete promise;

  return response;
}

void ConfigManager::handle_config_response_message(const mavlink_message_t &msg)
{
  mavlink_rosflight_config_status_t response_msg;
  mavlink_msg_rosflight_config_status_decode(&msg, &response_msg);
  uint8_t device = response_msg.device;

  ConfigResponse response;
  response.successful = response_msg.success;
  response.error_message = std::string(reinterpret_cast<char *>(response_msg.error_message));
  response.reboot_required = response_msg.reboot_required;

  for (size_t promise_index{0}; promise_index < config_response_promises_.size(); promise_index++)
    if (config_response_promises_[promise_index]->device == device)
    {
      config_response_promises_[promise_index]->promise.set_value(response);
      config_response_promises_.erase(config_response_promises_.begin() + promise_index);
      break;
    }
}

bool ConfigManager::is_valid_device(uint8_t device) const
{
  return (device < device_info_.size());
}

bool ConfigManager::is_valid_config(uint8_t device, uint8_t config) const
{
  return (device < device_info_.size() && config < device_info_[device].config_names.size());
}

std::string ConfigManager::get_device_name(uint8_t device) const
{
  if (is_valid_device(device))
    return device_info_[device].name;
  else
    return "Invalid device #" + std::to_string(static_cast<int>(device));
}

std::string ConfigManager::get_config_name(uint8_t device, uint8_t config) const
{
  if (is_valid_config(device, config))
    return device_info_[device].config_names[config];
  else
    return "Invalid configuration #" + std::to_string(static_cast<int>(config));
}

std::vector<std::string> ConfigManager::get_device_names() const
{
  std::vector<std::string> device_names;
  for(DeviceInfo device: device_info_)
    device_names.push_back(device.name);
  return device_names;
}

const std::vector<std::string> &ConfigManager::get_config_names(uint8_t device) const
{
  return device_info_[device].config_names;
}

std::tuple<bool, uint8_t> ConfigManager::get_device_from_str(const std::string &name) const
{
  if(is_uint8(name))
    return std::make_tuple(true, static_cast<uint8_t>(std::stoul(name)));
  std::string internal_name = make_internal_name(name);
  for (uint8_t i{0}; i < device_info_.size(); i++)
    if (internal_name == device_info_[i].internal_name)
      return std::make_tuple(true, i);
  return std::make_tuple(false, 0);
}

std::tuple<bool, uint8_t>
ConfigManager::get_config_from_str(uint8_t device, const std::string &name) const
{
  if(is_uint8(name))
    return std::make_tuple(true, static_cast<uint8_t>(std::stoul(name)));
  const std::vector<std::string> &configs = device_info_[device].config_names;
  std::string internal_name = make_internal_name(name);
  if (internal_name == "default")
    return std::make_tuple(true, 0);
  uint8_t word_match{0};
  unsigned int word_match_count{0};
  for (uint8_t i{0}; i < configs.size(); i++)
  {
    std::string config_internal_name = make_internal_name(configs[i]);
    if (internal_name == config_internal_name)
      return std::make_tuple(true, i);
    std::vector<std::string> config_words = get_words(config_internal_name);
    for (std::string config_word: config_words)
      if (config_word == internal_name)
      {
        word_match = i;
        word_match_count++;
        break;
      }
  }
  if (word_match_count == 1)
    return std::make_tuple(true, word_match);
  return std::make_tuple(false, 0);
}

void ConfigManager::request_config_info()
{
  ROS_INFO_ONCE("Requesting all configurations");
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_pack(1, 0, &msg, ROSFLIGHT_CMD_SEND_ALL_CONFIG_INFOS);
  comm_->send_message(msg);
  config_receive_timer_ = nh_.createTimer(ros::Duration(0.5),
                                          &ConfigManager::restart_config_info_request,
                                          this, true, true);
  
}

void ConfigManager::send_config_set_request(uint8_t device, uint8_t config)
{
  mavlink_message_t config_message;
  mavlink_msg_rosflight_config_pack(1, 0, &config_message, device, config);
  comm_->send_message(config_message);
}

void ConfigManager::handle_device_info_message(const mavlink_message_t &msg)
{
  mavlink_rosflight_device_info_t device_info;
  mavlink_msg_rosflight_device_info_decode(&msg, &device_info);
  uint8_t device = device_info.device;
  ROS_DEBUG("Device info recieved: \"%s\" #%d", device_info.name, device);
  if (device == device_info_.size())
    device_info_.resize(device + 1);
  else if (device > device_info_.size()) // This indicates that some earlier message has been missed
  {
    restart_config_info_request();
    return;
  }
  device_info_[device].name = std::string{reinterpret_cast<char *>(device_info.name)};
  device_info_[device].internal_name = make_internal_name(device_info_[device].name);
  device_info_[device].max_config = device_info.max_config;
  num_devices_ = device_info.num_devices;
  restart_config_receive_timer();
  if (device == num_devices_ - 1 && device_info.max_config == 0)
    finish_config_info_receive();
}

void ConfigManager::handle_config_info_message(const mavlink_message_t &msg)
{
  mavlink_rosflight_config_info_t config_info;
  mavlink_msg_rosflight_config_info_decode(&msg, &config_info);
  uint8_t device = config_info.device;
  uint8_t config = config_info.config;
  ROS_DEBUG("Config info recieved: \"%s\" #%d for device #%d", config_info.name, config, device);
  if (device >= device_info_.size()) // Some previous device info message is missing
  {
    restart_config_info_request();
    return;
  }
  if (config == device_info_[device].config_names.size())
    device_info_[device].config_names.resize(config + 1);
  else if (config > device_info_[device].config_names.size()) // Some previous config info message is missing
  {
    restart_config_info_request();
    return;
  }
  device_info_[device].config_names[config] = std::string{reinterpret_cast<char *>(config_info.name)};
  restart_config_receive_timer();
  if (device == num_devices_ - 1 && config == device_info_[device].max_config)
    finish_config_info_receive();
}

std::string ConfigManager::make_internal_name(const std::string &name)
{
  std::string internal_name = boost::algorithm::to_lower_copy(name);
  for (size_t i{0}; i < internal_name.length(); i++)
    if (internal_name[i] == ' ')
      internal_name[i] = '_';
  return internal_name;
}

std::vector<std::string> ConfigManager::get_words(const std::string &internal_name)
{
  std::vector<std::string> words;
  size_t start_index{0};
  constexpr int min_word_size{3};
  while (start_index < internal_name.length())
  {
    size_t next_index = internal_name.find('_', start_index);
    if (next_index == std::string::npos)
      next_index = internal_name.length();
    if (next_index - start_index >= min_word_size)
    {
      std::string word = internal_name.substr(start_index, next_index - start_index);
      words.push_back(word);
    }
    start_index = next_index + 1;
  }
  return words;
}

bool ConfigManager::is_uint8(const std::string &str)
{
  if (str.find_first_not_of("0123456789") != std::string::npos)
    return false;
  unsigned int value = std::stoul(str);
  return value <= UINT8_MAX;
}
} // namespace mavrosflight