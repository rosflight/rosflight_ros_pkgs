/*
 * Copyright 2017 James Jackson
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef fcu_common_joy_JOY_H_
#define fcu_common_joy_JOY_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <fcu_common/Command.h>
#include "gazebo_msgs/ModelState.h"

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

  fcu_common::Command command_msg_;
  fcu_common::Command autopilot_command_;
  //  fcu_common::ExtendedCommand extended_command_msg_;
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
  void APCommandCallback(const fcu_common::CommandConstPtr &msg);
  void Publish();

public:
  Joy();
};

#endif  // fcu_common_joy_JOY_H_
