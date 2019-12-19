#include <rosflight/mavrosflight/config_manager.h>
#include <boost/algorithm/string.hpp>
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
      if (msg.msgid >= 200)
        std::cout << static_cast<int>(msg.msgid) << std::endl;
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

std::tuple<bool, uint8_t> mavrosflight::ConfigManager::get_configuration(uint8_t device)
{
  config_promise_t *promise = new config_promise_t;
  promise->device = device;
  promises_.push_back(promise);

  std::future<uint8_t> future = promise->promise.get_future();
  send_config_request(device);
  std::future_status result = future.wait_for(timeout);
  bool success;
  uint8_t config{0};
  if (result == std::future_status::ready)
  {
    config = future.get();
    success = true;
  } else
    success = false;
  delete promise;
  return std::make_tuple(success, config);
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
  return true;
}

bool mavrosflight::ConfigManager::is_valid_device(uint8_t device) const
{
  return (device < device_info_.size());
}

bool mavrosflight::ConfigManager::is_valid_config(uint8_t device, uint8_t config) const
{
  return (device < device_info_.size() && config < device_info_[device].config_names.size());
}

std::string mavrosflight::ConfigManager::get_device_name(uint8_t device) const
{
  if (is_valid_device(device))
    return device_info_[device].name;
  else
    return "";
}

std::string mavrosflight::ConfigManager::get_config_name(uint8_t device, uint8_t config) const
{
  if (is_valid_config(device, config))
    return device_info_[device].config_names[config];
  else
    return "";
}

std::tuple<bool, uint8_t> mavrosflight::ConfigManager::get_device_from_str(const std::string &name) const
{
  std::string internal_name = make_internal_name(name);
  for (uint8_t i{0}; i < device_info_.size(); i++)
    if (internal_name == device_info_[i].internal_name)
      return std::make_tuple(true, i);
  return std::make_tuple(false, 0);
}

std::tuple<bool, uint8_t>
mavrosflight::ConfigManager::get_config_from_str(uint8_t device, const std::string &name) const
{
  const std::vector<std::string> &configs = device_info_[device].config_names;
  std::string internal_name = make_internal_name(name);
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
  if (device >= device_info_.size())
    device_info_.resize(device + 1);
  device_info_[device].name = std::string{reinterpret_cast<char *>(device_info.name)};
  device_info_[device].internal_name = make_internal_name(device_info_[device].name);
  device_info_[device].max_value = device_info.max_value;
  std::cout << "Device info recieved: " << static_cast<int>(device) << ',' << device_info_[device].name << std::endl;
  std::cout << device_info_[device].internal_name << std::endl;
}

void mavrosflight::ConfigManager::handle_config_info_message(const mavlink_message_t &msg)
{
  mavlink_rosflight_config_info_t config_info;
  mavlink_msg_rosflight_config_info_decode(&msg, &config_info);
  uint8_t device = config_info.device;
  uint8_t config = config_info.config;
  if (device >= device_info_.size())
    device_info_.resize(device + 1);
  if (config >= device_info_[device].config_names.size())
    device_info_[device].config_names.resize(config + 1);
  device_info_[device].config_names[config] = std::string{reinterpret_cast<char *>(config_info.name)};
  std::cout << "Config info recieved: " << static_cast<int>(device) << ',' << static_cast<int>(config) << ','
            << device_info_[device].config_names[config] << std::endl;
}

std::string mavrosflight::ConfigManager::make_internal_name(const std::string &name)
{
  std::string internal_name = boost::algorithm::to_lower_copy(name);
  for (size_t i{0}; i < internal_name.length(); i++)
    if (internal_name[i] == ' ')
      internal_name[i] = '_';
  return internal_name;
}

std::vector<std::string> mavrosflight::ConfigManager::get_words(const std::string &internal_name)
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

bool mavrosflight::ConfigManager::is_uint8(const std::string &str)
{
  if(str.find_first_not_of("0123456789") != std::string::npos)
    return false;
  unsigned int value = std::stoul(str);
  return value <= UINT8_MAX;
}
