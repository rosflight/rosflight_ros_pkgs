/**
 * \file param_manager.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_PARAM_MANAGER_H
#define MAVROSFLIGHT_PARAM_MANAGER_H

#include <mavrosflight/mavlink_bridge.h>
#include <mavrosflight/mavlink_listener_interface.h>
#include <mavrosflight/mavlink_serial.h>
#include <mavrosflight/param.h>
#include <mavrosflight/param_listener_interface.h>

#include <map>
#include <string>
#include <vector>

namespace mavrosflight
{

class ParamManager : public MavlinkListenerInterface
{
public:
  ParamManager(MavlinkSerial * const serial);
  ~ParamManager();

  virtual void handle_mavlink_message(const mavlink_message_t &msg);

  bool unsaved_changes();

  bool get_param_value(std::string name, double *value);
  bool set_param_value(std::string name, double value);
  bool write_params();

  void register_param_listener(ParamListenerInterface *listener);
  void unregister_param_listener(ParamListenerInterface *listener);

  bool save_to_file(std::string filename);
  bool load_from_file(std::string filename);

  int get_param_count();
  bool got_all_params();

  void request_param_list();

private:

  void handle_param_value_msg(const mavlink_message_t &msg);
  void handle_command_ack_msg(const mavlink_message_t &msg);

  bool is_param_id(std::string name);

  std::vector<ParamListenerInterface*> listeners_;

  MavlinkSerial *serial_;
  std::map<std::string, Param> params_;

  bool unsaved_changes_;
  bool write_request_in_progress_;

  bool first_param_received_;
  size_t param_count_;
  bool *received_;
  bool initialized_;
  bool got_all_params_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_PARAM_MANAGER_H
