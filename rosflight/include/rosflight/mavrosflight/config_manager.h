#include <vector>
#include <future>
#include <chrono>
#include <string>
#include <utility>

#include <ros/ros.h>

#include "mavlink_listener_interface.h"
#include "mavlink_comm.h"

namespace mavrosflight
{

class ConfigManager : public MavlinkListenerInterface
{
public:

struct ConfigResponse
{
  bool successful;
  bool reboot_required;
  std::string error_message;
};

ConfigManager(MavlinkComm *const comm);
~ConfigManager();
void handle_mavlink_message(const mavlink_message_t &msg) override;
std::tuple<bool, uint8_t> get_configuration(uint8_t device);
ConfigResponse set_configuration(uint8_t device, uint8_t config);
void request_config_info();

std::tuple<bool, uint8_t> get_device_from_str(const std::string &name) const;
std::tuple<bool, uint8_t> get_config_from_str(uint8_t device, const std::string &name) const;
bool is_valid_device(uint8_t device) const;
bool is_valid_config(uint8_t device, uint8_t config) const;
std::string get_device_name(uint8_t device) const;
std::string get_config_name(uint8_t device, uint8_t config) const;
std::vector<std::string> get_device_names() const;
const std::vector<std::string> &get_config_names(uint8_t device) const;

static constexpr std::chrono::milliseconds TIMEOUT = std::chrono::milliseconds(500);

private:

struct ConfigPromise
{
  std::promise<uint8_t> promise;
  uint8_t device;
};
MavlinkComm *const comm_;
std::vector<ConfigPromise *> promises_;


struct ConfigResponsePromise
{
  std::promise<ConfigResponse> promise;
  uint8_t device;
  uint8_t config;
};
std::vector<ConfigResponsePromise *> config_response_promises_;

struct DeviceInfo
{
  std::string name;
  std::string internal_name;
  uint8_t max_config;
  std::vector<std::string> config_names;
};
std::vector<DeviceInfo> device_info_;

ros::NodeHandle nh_;
ros::Timer config_receive_timer_;
uint8_t num_devices_{0};

void handle_config_message(const mavlink_message_t &msg);
void handle_device_info_message(const mavlink_message_t &msg);
void handle_config_info_message(const mavlink_message_t &msg);
void handle_config_response_message(const mavlink_message_t &msg);
void send_config_get_request(uint8_t device);
void send_config_set_request(uint8_t device, uint8_t config);
// In case of an error, shows a message and restarts getting configuration info
void restart_config_info_request();
void restart_config_info_request(const ros::TimerEvent &event); // for use in timers
void restart_config_receive_timer();
void finish_config_info_receive();

static std::string make_internal_name(const std::string &name);
static std::vector<std::string> get_words(const std::string &internal_name);
static bool is_uint8(const std::string &str);

}; // class ConfigManager
} // namespace mavrosflight
