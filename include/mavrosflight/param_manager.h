#ifndef MAVROSFLIGHT_PARAM_MANAGER_H
#define MAVROSFLIGHT_PARAM_MANAGER_H

#include <mavrosflight/mavlink_bridge.h>
#include <mavrosflight/mavlink_listener_interface.h>
#include <mavrosflight/mavlink_serial.h>
#include <mavrosflight/param.h>

#include <string>
#include <map>

namespace mavrosflight
{

class ParamManager : public MavlinkListenerInterface
{
public:
  ParamManager(MavlinkSerial * const serial);
  ~ParamManager();

  virtual void handle_mavlink_message(const mavlink_message_t &msg);

  bool is_param_id(std::string name);
  bool unsaved_changes();

  void handle_param_value_msg(const mavlink_message_t &msg);
  void handle_command_ack_msg(const mavlink_message_t &msg);
  bool set_param_value(std::string name, double value);
  void write_params();

private:
  MavlinkSerial *serial_;
  std::map<std::string, Param> params_;

  bool unsaved_changes_;
  bool write_request_in_progress_;

  bool first_param_received_;
  size_t param_count_;
  bool *received_;
  bool initialized_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_PARAM_MANAGER_H
