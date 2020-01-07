#include <vector>
#include <future>
#include <chrono>
#include <string>
#include <utility>

#include "mavlink_listener_interface.h"
#include "mavlink_comm.h"

namespace mavrosflight
{

  class ConfigManager : public MavlinkListenerInterface
  {
  public:
    typedef struct
    {
      bool successful;
      bool reboot_required;
      std::string error_message;
    } config_response_t;
    ConfigManager(MavlinkComm *const comm);
    ~ConfigManager();
    void handle_mavlink_message(const mavlink_message_t &msg) override;
    std::tuple<bool, uint8_t> get_configuration(uint8_t device);
    config_response_t set_configuration(uint8_t device, uint8_t config);
    void request_config_info();

    std::tuple<bool, uint8_t> get_device_from_str(const std::string &name) const;
    std::tuple<bool, uint8_t> get_config_from_str(uint8_t device, const std::string &name) const;
    bool is_valid_device(uint8_t device) const;
    bool is_valid_config(uint8_t device, uint8_t config) const;
    std::string get_device_name(uint8_t device) const;
    std::string get_config_name(uint8_t device, uint8_t config) const;

    static constexpr std::chrono::milliseconds timeout{500};
  private:
    typedef struct
    {
      std::promise<uint8_t> promise;
      uint8_t device;
    } config_promise_t;
    MavlinkComm *const comm_;
    std::vector<config_promise_t *> promises_;


    typedef struct
    {
      std::promise<config_response_t> promise;
      uint8_t device;
      uint8_t config;
    } config_response_promise_t;
    std::vector<config_response_promise_t *> config_response_promises_;

    typedef struct
    {
      std::string name;
      std::string internal_name;
      uint8_t max_value;
      std::vector<std::string> config_names;
    } device_info_t;
    std::vector<device_info_t> device_info_;

    void handle_config_message(const mavlink_message_t &msg);
    void handle_device_info_message(const mavlink_message_t &msg);
    void handle_config_info_message(const mavlink_message_t &msg);
    void handle_config_response_message(const mavlink_message_t &msg);
    void send_config_get_request(uint8_t device);
    void send_config_set_request(uint8_t device, uint8_t config);

    static std::string make_internal_name(const std::string &name);
    static std::vector<std::string> get_words(const std::string &internal_name);
    static bool is_uint8(const std::string &str);

  }; // class ConfigManager
} // namespace mavrosflight
