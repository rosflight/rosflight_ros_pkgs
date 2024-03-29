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

#include <rosflight_io/mavrosflight/mavlink_bridge.hpp>
#include <rosflight_io/mavrosflight/mavlink_comm.hpp>
#include <rosflight_io/mavrosflight/mavlink_listener_interface.hpp>
#include <rosflight_io/mavrosflight/param.hpp>
#include <rosflight_io/mavrosflight/param_listener_interface.hpp>

#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mavrosflight
{
class ParamManager : public MavlinkListenerInterface
{
public:
  ParamManager(MavlinkComm * comm, rclcpp::Node * node);
  ~ParamManager();

  void handle_mavlink_message(const mavlink_message_t & msg) override;

  bool unsaved_changes() const;

  bool get_param_value(const std::string & name, double * value);
  bool set_param_value(const std::string & name, double value);
  bool write_params();

  void register_param_listener(ParamListenerInterface * listener);
  void unregister_param_listener(ParamListenerInterface * listener);

  bool save_to_file(const std::string & filename);
  bool load_from_file(const std::string & filename);

  int get_num_params() const;
  int get_params_received() const;
  bool got_all_params() const;

  void request_params();

private:
  void request_param_list();
  void request_param(int index);

  void handle_param_value_msg(const mavlink_message_t & msg);
  void handle_command_ack_msg(const mavlink_message_t & msg);

  bool is_param_id(const std::string & name);

  std::vector<ParamListenerInterface *> listeners_;

  rclcpp::Node * const node_;
  MavlinkComm * const comm_;
  std::map<std::string, Param> params_;

  bool unsaved_changes_;
  bool write_request_in_progress_;

  bool first_param_received_;
  int num_params_;
  int received_count_;
  bool * received_;
  bool got_all_params_;

  std::deque<mavlink_message_t> param_set_queue_;
  rclcpp::TimerBase::SharedPtr param_set_timer_;
  bool param_set_in_progress_;
  void param_set_timer_callback();
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_PARAM_MANAGER_H
