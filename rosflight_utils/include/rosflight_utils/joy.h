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

#ifndef ROSFLIGHT_COMMON_JOY_JOY_H
#define ROSFLIGHT_COMMON_JOY_JOY_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <rosflight_msgs/Command.h>
#include <gazebo_msgs/ModelState.h>

struct Axes
{
  int x;
  int y;
  int F;
  int z;
  int x_direction;
  int y_direction;
  int F_direction;
  int z_direction;
};

struct Max
{
  double roll;
  double pitch;

  double roll_rate;
  double pitch_rate;
  double yaw_rate;

  double aileron;
  double elevator;
  double rudder;

  double xvel;
  double yvel;
  double zvel;
};

struct Button
{
  int index;
  bool prev_value;
};

struct Buttons
{
  Button fly;
  Button mode;
  Button reset;
  Button pause;
  Button override;
};

class Joy
{
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

private:
  ros::NodeHandle nh_;
  ros::Publisher command_pub_;
  ros::Subscriber autopilot_command_sub_;
  ros::Subscriber joy_sub_;

  std::string namespace_;
  std::string command_topic_;
  std::string autopilot_command_topic_;

  std::string mav_name_;
  std::string gazebo_ns_;

  Axes axes_;

  bool override_autopilot_ = true;
  bool paused = true;
  double equilibrium_thrust_;

  rosflight_msgs::Command command_msg_;
  rosflight_msgs::Command autopilot_command_;
  sensor_msgs::Joy current_joy_;

  Max max_;
  Buttons buttons_;
  geometry_msgs::Pose reset_pose_;
  geometry_msgs::Twist reset_twist_;

  double current_altitude_setpoint_;
  double current_x_setpoint_;
  double current_y_setpoint_;
  double current_yaw_setpoint_;
  double last_time_;


  double current_yaw_vel_;
  double v_yaw_step_;
  double mass_;

  void StopMav();
  void ResetMav();
  void PauseSimulation();
  void ResumeSimulation();

  void JoyCallback(const sensor_msgs::JoyConstPtr &msg);
  void APCommandCallback(const rosflight_msgs::CommandConstPtr &msg);
  void Publish();

public:
  Joy();
};

#endif  // ROSFLIGHT_COMMON_JOY_JOY_H
