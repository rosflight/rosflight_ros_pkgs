/*
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file param_manager.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_PARAM_MANAGER_H
#define MAVROSFLIGHT_PARAM_MANAGER_H

#include <rosflight/mavrosflight/mavlink_bridge.h>
#include <rosflight/mavrosflight/mavlink_comm.h>
#include <rosflight/mavrosflight/mavlink_listener_interface.h>
#include <rosflight/mavrosflight/param.h>
#include <rosflight/mavrosflight/param_listener_interface.h>

#include <deque>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>

namespace mavrosflight
{

class ParamManager : public MavlinkListenerInterface
{
public:
  ParamManager(MavlinkComm * const comm);
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

  int get_num_params();
  int get_params_received();
  bool got_all_params();

  void request_params();

private:

  void request_param_list();
  void request_param(int index);

  void handle_param_value_msg(const mavlink_message_t &msg);
  void handle_command_ack_msg(const mavlink_message_t &msg);

  bool is_param_id(std::string name);

  std::vector<ParamListenerInterface*> listeners_;

  MavlinkComm *comm_;
  std::map<std::string, Param> params_;

  bool unsaved_changes_;
  bool write_request_in_progress_;

  bool first_param_received_;
  size_t num_params_;
  size_t received_count_;
  bool *received_;
  bool got_all_params_;

  ros::NodeHandle nh_;
  std::deque<mavlink_message_t> param_set_queue_;
  ros::Timer param_set_timer_;
  bool param_set_in_progress_;
  void param_set_timer_callback(const ros::TimerEvent &event);
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_PARAM_MANAGER_H
