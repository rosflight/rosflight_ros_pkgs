#include <vector>
#include <future>
#include <chrono>

#include "mavlink_listener_interface.h"
#include "mavlink_comm.h"

namespace mavrosflight
{

  class ConfigManager : public MavlinkListenerInterface
  {
  public:
    ConfigManager(MavlinkComm *const comm);
    ~ConfigManager();
    void handle_mavlink_message(const mavlink_message_t &msg) override;
    bool get_configuration(uint8_t device, uint8_t &config);
    void set_configuration(uint8_t device, uint8_t config);

    static constexpr std::chrono::milliseconds timeout{500};
  private:
    typedef struct
    {
      std::promise<uint8_t> promise;
      uint8_t device;
    } config_promise_t;
    MavlinkComm *const comm_;
    std::vector<config_promise_t *> promises_;

    void handle_config_message(const mavlink_message_t &msg);
    void send_config_request(uint8_t device);
    void send_config(uint8_t device, uint8_t config);

  }; // class ConfigManager
} // namespace mavrosflight
