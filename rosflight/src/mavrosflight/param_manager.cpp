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

#include <fstream>
#include <functional>

#include <rosflight/mavrosflight/param_manager.hpp>
#include <yaml-cpp/yaml.h>

namespace mavrosflight
{
ParamManager::ParamManager(MavlinkComm * const comm, rclcpp::Node * const node)
    : node_(node), comm_(comm), unsaved_changes_(false), write_request_in_progress_(false),
      first_param_received_(false), num_params_(0), received_count_(0), received_(nullptr),
      got_all_params_(false), param_set_in_progress_(false)
{
  comm_->register_mavlink_listener(this);

  param_set_timer_ =
    node_->create_wall_timer(std::chrono::milliseconds(10),
                             std::bind(&ParamManager::param_set_timer_callback, this), nullptr);
}

ParamManager::~ParamManager()
{
  if (first_param_received_) { delete[] received_; }
}

void ParamManager::handle_mavlink_message(const mavlink_message_t & msg)
{
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_PARAM_VALUE:
      handle_param_value_msg(msg);
      break;
    case MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK:
      handle_command_ack_msg(msg);
      break;
  }
}

bool ParamManager::unsaved_changes() const { return unsaved_changes_; }

bool ParamManager::get_param_value(const std::string & name, double * value)
{
  if (is_param_id(name)) {
    *value = params_[name].getValue();
    return true;
  } else {
    *value = 0.0;
    return false;
  }
}

bool ParamManager::set_param_value(const std::string & name, double value)
{
  if (is_param_id(name)) {
    mavlink_message_t msg;
    params_[name].requestSet(value, &msg);

    param_set_queue_.push_back(msg);
    if (!param_set_in_progress_) {
      param_set_timer_->reset();
      param_set_in_progress_ = true;
    }

    return true;
  } else {
    return false;
  }
}

bool ParamManager::write_params()
{
  if (!write_request_in_progress_) {
    mavlink_message_t msg;
    uint8_t sysid = 1;
    uint8_t compid = 1;
    mavlink_msg_rosflight_cmd_pack(sysid, compid, &msg, ROSFLIGHT_CMD_WRITE_PARAMS);
    comm_->send_message(msg);

    write_request_in_progress_ = true;
    return true;
  } else {
    return false;
  }
}

void ParamManager::register_param_listener(ParamListenerInterface * listener)
{
  if (listener == nullptr) { return; }

  bool already_registered = false;
  for (auto & item : listeners_) {
    if (listener == item) {
      already_registered = true;
      break;
    }
  }

  if (!already_registered) { listeners_.push_back(listener); }
}

void ParamManager::unregister_param_listener(ParamListenerInterface * listener)
{
  if (listener == nullptr) { return; }

  for (int i = 0; i < (int) listeners_.size(); i++) {
    if (listener == listeners_[i]) {
      listeners_.erase(listeners_.begin() + i);
      i--;
    }
  }
}

bool ParamManager::save_to_file(const std::string & filename)
{
  // build YAML document
  YAML::Emitter yaml;
  yaml << YAML::BeginSeq;
  std::map<std::string, Param>::iterator it;
  for (it = params_.begin(); it != params_.end(); it++) {
    yaml << YAML::Flow;
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "name" << YAML::Value << it->second.getName();
    yaml << YAML::Key << "type" << YAML::Value << (int) it->second.getType();
    yaml << YAML::Key << "value" << YAML::Value << it->second.getValue();
    yaml << YAML::EndMap;
  }
  yaml << YAML::EndSeq;

  // write to file
  try {
    std::ofstream fout;
    fout.open(filename.c_str());
    fout << yaml.c_str();
    fout.close();
  } catch (...) {
    return false;
  }

  return true;
}

bool ParamManager::load_from_file(const std::string & filename)
{
  try {
    YAML::Node root = YAML::LoadFile(filename);
    assert(root.IsSequence());

    for (auto && item : root) {
      if (item.IsMap() && item["name"] && item["type"] && item["value"]) {
        if (is_param_id(item["name"].as<std::string>())) {
          Param param = params_.find(item["name"].as<std::string>())->second;
          if ((MAV_PARAM_TYPE) item["type"].as<int>() == param.getType()) {
            set_param_value(item["name"].as<std::string>(), item["value"].as<double>());
          }
        }
      }
    }

    return true;
  } catch (...) {
    return false;
  }
}

void ParamManager::request_params()
{
  if (!first_param_received_) {
    request_param_list();
  } else {
    for (int i = 0; i < num_params_; i++) {
      if (!received_[i]) { request_param(i); }
    }
  }
}

void ParamManager::request_param_list()
{
  mavlink_message_t param_list_msg;
  mavlink_msg_param_request_list_pack(1, 50, &param_list_msg, 1, MAV_COMP_ID_ALL);
  comm_->send_message(param_list_msg);
}

void ParamManager::request_param(int index)
{
  mavlink_message_t param_request_msg;
  char empty[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = {0};
  mavlink_msg_param_request_read_pack(1, 50, &param_request_msg, 1, MAV_COMP_ID_ALL, empty,
                                      (int16_t) index);
  comm_->send_message(param_request_msg);
}

void ParamManager::handle_param_value_msg(const mavlink_message_t & msg)
{
  mavlink_param_value_t param;
  mavlink_msg_param_value_decode(&msg, &param);

  if (!first_param_received_) {
    first_param_received_ = true;
    num_params_ = param.param_count;
    received_ = new bool[num_params_];
    for (int i = 0; i < num_params_; i++) { received_[i] = false; }
  }

  // ensure null termination of name
  char c_name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
  memcpy(c_name, param.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
  c_name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

  std::string name(c_name);

  if (!is_param_id(name)) // if we haven't received this param before, add it
  {
    params_[name] = Param(param);
    received_[param.param_index] = true;

    // increase the param count
    received_count_++;
    if (received_count_ == num_params_) { got_all_params_ = true; }

    for (auto & listener : listeners_) {
      listener->on_new_param_received(name, params_[name].getValue());
    }
  } else // otherwise check if we have new unsaved changes as a result of a param set request
  {
    if (params_[name].handleUpdate(param)) {
      unsaved_changes_ = true;
      for (auto & listener : listeners_) {
        listener->on_param_value_updated(name, params_[name].getValue());
        listener->on_params_saved_change(unsaved_changes_);
      }
    }
  }
}

void ParamManager::handle_command_ack_msg(const mavlink_message_t & msg)
{
  if (write_request_in_progress_) {
    mavlink_rosflight_cmd_ack_t ack;
    mavlink_msg_rosflight_cmd_ack_decode(&msg, &ack);

    if (ack.command == ROSFLIGHT_CMD_WRITE_PARAMS) {
      write_request_in_progress_ = false;
      if (ack.success == ROSFLIGHT_CMD_SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Param write succeeded");
        unsaved_changes_ = false;

        for (auto & listener : listeners_) { listener->on_params_saved_change(unsaved_changes_); }
      } else {
        RCLCPP_INFO(node_->get_logger(),
                    "Param write failed - maybe disarm the aircraft and try again?");
        write_request_in_progress_ = false;
        unsaved_changes_ = true;
      }
    }
  }
}

bool ParamManager::is_param_id(const std::string & name)
{
  return (params_.find(name) != params_.end());
}

int ParamManager::get_num_params() const
{
  if (first_param_received_) {
    return num_params_;
  } else {
    return 0;
  }
}

int ParamManager::get_params_received() const { return received_count_; }

bool ParamManager::got_all_params() const { return got_all_params_; }

void ParamManager::param_set_timer_callback()
{
  if (param_set_queue_.empty()) {
    param_set_timer_->cancel();
    param_set_in_progress_ = false;
  } else {
    comm_->send_message(param_set_queue_.front());
    param_set_queue_.pop_front();
  }
}

} // namespace mavrosflight
