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

#include <gazebo_msgs/srv/set_model_state.hpp>
#include <rosflight_utils/joy.h>
#include <std_srvs/srv/empty.hpp>

using std::placeholders::_1;

Joy::Joy() : Node("rosflight_utils_joy")
{
  command_topic_ = this->get_parameter_or<std::string>("command_topic", "command");
  autopilot_command_topic_ = this->get_parameter_or<std::string>("autopilot_command_topic", "autopilot_command");

  // Get global parameters
  mass_ = this->get_parameter_or<double>("mass", 3.61);
  double max_thrust = this->get_parameter_or<double>("max_F", 64.50);
  mav_name_ = this->get_parameter_or<std::string>("mav_name", "shredder");
  equilibrium_thrust_ = (mass_ * 9.80665) / max_thrust;

  // Get Parameters from joystick configuration yaml
  gazebo_ns_ = this->get_parameter_or<std::string>("gazebo_namespace", "");
  axes_.x = this->get_parameter_or<int>("x_axis", 1);
  axes_.y = this->get_parameter_or<int>("y_axis", 2);
  axes_.F = this->get_parameter_or<int>("F_axis", 0);
  axes_.z = this->get_parameter_or<int>("z_axis", 4);

  axes_.x_direction = this->get_parameter_or<int>("x_sign", 1);
  axes_.y_direction = this->get_parameter_or<int>("y_sign", 1);
  axes_.F_direction = this->get_parameter_or<int>("F_sign", -1);
  axes_.z_direction = this->get_parameter_or<int>("z_sign", 1);

  max_.aileron = this->get_parameter_or<double>("max_aileron", 15.0 * M_PI / 180.0);
  max_.elevator = this->get_parameter_or<double>("max_elevator", 25.0 * M_PI / 180.0);
  max_.rudder = this->get_parameter_or<double>("max_rudder", 15.0 * M_PI / 180.0);

  max_.roll_rate = this->get_parameter_or<double>("max_roll_rate", 360.0 * M_PI / 180.0);
  max_.pitch_rate = this->get_parameter_or<double>("max_pitch_rate", 360.0 * M_PI / 180.0);
  max_.yaw_rate = this->get_parameter_or<double>("max_yaw_rate", 360.0 * M_PI / 180.0);

  max_.roll = this->get_parameter_or<double>("max_roll_angle", 45.0 * M_PI / 180.0);
  max_.pitch = this->get_parameter_or<double>("max_pitch_angle", 45.0 * M_PI / 180.0);

  max_.xvel = this->get_parameter_or<double>("max_xvel", 1.5);
  max_.yvel = this->get_parameter_or<double>("max_yvel", 1.5);
  max_.zvel = this->get_parameter_or<double>("max_zvel", 1.5);
  command_msg_.mode = this->get_parameter_or<int>(
    "control_mode", (int)rosflight_msgs::msg::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE);

  reset_pose_.position.x = this->get_parameter_or<double>("reset_pos_x", 0.0);
  reset_pose_.position.y = this->get_parameter_or<double>("reset_pos_y", 0.0);
  reset_pose_.position.z = this->get_parameter_or<double>("reset_pos_z", 0.0);
  reset_pose_.orientation.x = this->get_parameter_or<double>("reset_orient_x", 0.0);
  reset_pose_.orientation.y = this->get_parameter_or<double>("reset_orient_y", 0.0);
  reset_pose_.orientation.z = this->get_parameter_or<double>("reset_orient_z", 0.0);
  reset_pose_.orientation.w = this->get_parameter_or<double>("reset_orient_w", 0.0);
  reset_twist_.linear.x = this->get_parameter_or<double>("reset_linear_twist_x", 0.0);
  reset_twist_.linear.y = this->get_parameter_or<double>("reset_linear_twist_y", 0.0);
  reset_twist_.linear.z = this->get_parameter_or<double>("reset_linear_twist_z", 0.0);
  reset_twist_.angular.x = this->get_parameter_or<double>("reset_angular_twist_x", 0.0);
  reset_twist_.angular.y = this->get_parameter_or<double>("reset_angular_twist_y", 0.0);
  reset_twist_.angular.z = this->get_parameter_or<double>("reset_angular_twist_z", 0.0);

  // Sets which buttons are tied to which commands
  buttons_.fly.index = this->get_parameter_or<int>("button_takeoff", 0);
  buttons_.mode.index = this->get_parameter_or<int>("button_mode", 1);
  buttons_.reset.index = this->get_parameter_or<int>("button_reset", 9);
  buttons_.pause.index = this->get_parameter_or<int>("button_pause", 8);
  buttons_.override.index = this->get_parameter_or<int>("button_override", 8);

  command_pub_ = this->create_publisher<rosflight_msgs::msg::Command>(command_topic_, 10);

  command_msg_.x = 0;
  command_msg_.y = 0;
  command_msg_.z = 0;
  command_msg_.f = 0;
  command_msg_.ignore = 0xFF;

  current_yaw_vel_ = 0;

  namespace_ = this->get_namespace();
  autopilot_command_sub_ = this->create_subscription<rosflight_msgs::msg::Command>(
    autopilot_command_topic_, 10, std::bind(&Joy::APCommandCallback, this, _1));
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&Joy::JoyCallback, this, _1));
  buttons_.mode.prev_value = 0;
  buttons_.reset.prev_value = 0;

  current_altitude_setpoint_ = current_x_setpoint_ = current_y_setpoint_ = 0.0;
}

void Joy::StopMav()
{
  command_msg_.x = 0;
  command_msg_.y = 0;
  command_msg_.z = 0;
  command_msg_.f = 0;
}

/* Resets the mav back to origin */
void Joy::ResetMav()
{
  RCLCPP_INFO(this->get_logger(), "Mav position reset.");

  gazebo_msgs::msg::ModelState modelstate;
  modelstate.model_name = (std::string)mav_name_;
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = reset_pose_;
  modelstate.twist = reset_twist_;

  auto client = this->create_client<gazebo_msgs::srv::SetModelState>("/gazebo/set_model_state");
  auto setModelStateRequest = std::make_shared<gazebo_msgs::srv::SetModelState::Request>();
  setModelStateRequest->model_state = modelstate;
  rclcpp::spin_until_future_complete(rclcpp::Node::SharedPtr(this),
                                     client->async_send_request(setModelStateRequest));
}

// Pauses the gazebo physics and time
void Joy::PauseSimulation()
{
  RCLCPP_INFO(this->get_logger(), "Simulation paused.");

  auto client = this->create_client<std_srvs::srv::Empty>("/gazebo/pause_physics");
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = client->async_send_request(request);
  auto node_ptr = rclcpp::Node::SharedPtr(this);
  rclcpp::spin_until_future_complete(node_ptr, result);
}

// Resumes the gazebo physics and time
void Joy::ResumeSimulation()
{
  RCLCPP_INFO(this->get_logger(), "Simulation resumed.");

  auto client = this->create_client<std_srvs::srv::Empty>("/gazebo/unpause_physics");
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = client->async_send_request(request);
  auto node_ptr = rclcpp::Node::SharedPtr(this);
  rclcpp::spin_until_future_complete(node_ptr, result);
  last_time_ = this->get_clock()->now().seconds();
}

void Joy::APCommandCallback(rosflight_msgs::msg::Command::ConstSharedPtr msg)
{
  autopilot_command_ = *msg;
}

void Joy::JoyCallback(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  double dt = this->get_clock()->now().seconds() - last_time_;
  last_time_ = this->get_clock()->now().seconds();

  current_joy_ = *msg;

  // Perform button actions
  if (msg->buttons[buttons_.override.index] == 0 && buttons_.override.prev_value == 1)
  {
    override_autopilot_ = true;
  }
  buttons_.override.prev_value = msg->buttons[buttons_.override.index];

  // Resets the mav to the origin
  if (msg->buttons[buttons_.reset.index] == 0 && buttons_.reset.prev_value == 1) // button release
  {
    ResetMav();
  }
  buttons_.reset.prev_value = msg->buttons[buttons_.reset.index];

  // Pauses/Unpauses the simulation
  if (msg->buttons[buttons_.pause.index] == 0 && buttons_.pause.prev_value == 1) // button release
  {
    if (!paused)
    {
      PauseSimulation();
      paused = true;
    }
    else
    {
      ResumeSimulation();
      paused = false;
    }
  }
  buttons_.pause.prev_value = msg->buttons[buttons_.pause.index];

  if (msg->buttons[buttons_.mode.index] == 0 && buttons_.mode.prev_value == 1)
  {
    command_msg_.mode = (command_msg_.mode + 1) % 6;
if (command_msg_.mode == rosflight_msgs::msg::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE)
    {
      RCLCPP_INFO(this->get_logger(), "Rate Mode");
    }
    else if (command_msg_.mode == rosflight_msgs::msg::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
    {
      RCLCPP_INFO(this->get_logger(), "Angle Mode");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Passthrough");
    }
  }
  buttons_.mode.prev_value = msg->buttons[buttons_.mode.index];

  // calculate the output command from the joysticks
  if (override_autopilot_)
  {
    command_msg_.f = msg->axes[axes_.F] * axes_.F_direction;
    command_msg_.x = msg->axes[axes_.x] * axes_.x_direction;
    command_msg_.y = msg->axes[axes_.y] * axes_.y_direction;
    command_msg_.z = msg->axes[axes_.z] * axes_.z_direction;

    switch (command_msg_.mode)
    {
    case rosflight_msgs::msg::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
      command_msg_.x *= max_.roll_rate;
      command_msg_.y *= max_.pitch_rate;
      command_msg_.z *= max_.yaw_rate;
      if (command_msg_.f > 0.0)
      {
        command_msg_.f = equilibrium_thrust_ + (1.0 - equilibrium_thrust_) * command_msg_.f;
      }
      else
      {
        command_msg_.f = equilibrium_thrust_ + (equilibrium_thrust_)*command_msg_.f;
      }
      break;

    case rosflight_msgs::msg::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE:
      command_msg_.x *= max_.roll;
      command_msg_.y *= max_.pitch;
      command_msg_.z *= max_.yaw_rate;
      if (command_msg_.f > 0.0)
      {
        command_msg_.f = equilibrium_thrust_ + (1.0 - equilibrium_thrust_) * command_msg_.f;
      }
      else
      {
        command_msg_.f = equilibrium_thrust_ + (equilibrium_thrust_)*command_msg_.f;
      }
      break;
    }
  }
  else
  {
    command_msg_ = autopilot_command_;
  }

  Publish();
}

void Joy::Publish()
{
  command_pub_->publish(command_msg_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joy>());
  rclcpp::shutdown();
  return 0;
}
