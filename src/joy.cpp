/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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


#include "fcu_common/joy.h"


Joy::Joy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("command_topic", command_topic_, "command");
  pnh.param<std::string>("autopilot_command_topic", autopilot_command_topic_, "autopilot_command");

  // Default to Spektrum Transmitter on Interlink
  pnh.param<int>("x_axis", axes_.roll, 1);
  pnh.param<int>("y_axis", axes_.pitch, 2);
  pnh.param<int>("F_axis", axes_.thrust, 0);
  pnh.param<int>("z_axis", axes_.yaw, 4);

  pnh.param<int>("x_sign", axes_.roll_direction, -1);
  pnh.param<int>("y_sign", axes_.pitch_direction, 1);
  pnh.param<int>("F_sign", axes_.thrust_direction, -1);
  pnh.param<int>("z_sign", axes_.yaw_direction, 1);

  pnh.param<double>("max_thrust", max_.thrust, 74.676);  // [N]
  pnh.param<double>("max_altitude", max_.altitude, 10.0); // [m]
  pnh.param<double>("mass", mass_, 3.81);  // [N]

  pnh.param<double>("max_aileron", max_.aileron, 15.0*M_PI/180.0);
  pnh.param<double>("max_elevator", max_.elevator, 25.0*M_PI/180.0);
  pnh.param<double>("max_rudder", max_.rudder, 15.0*M_PI/180.0);

  pnh.param<double>("max_roll_rate", max_.roll_rate, 360.0*M_PI/180.0);
  pnh.param<double>("max_pitch_rate", max_.pitch_rate, 360.0*M_PI/180.0);
  pnh.param<double>("max_yaw_rate", max_.yaw_rate, 360.0*M_PI/180.0);

  pnh.param<double>("max_roll_angle", max_.roll, 45.0*M_PI/180.0);
  pnh.param<double>("max_pitch_angle", max_.pitch, 45.0*M_PI/180.0);

  pnh.param<int>("button_takeoff_", buttons_.fly.index, 0);
  pnh.param<int>("button_mode_", buttons_.mode.index, 1);

  command_pub_ = nh_.advertise<fcu_common::Command>(command_topic_,10);
  extended_command_pub_ = nh_.advertise<fcu_common::ExtendedCommand>("extended_"+command_topic_, 10);

//  ROS_ERROR_STREAM("mass = " << mass_ <<" max_thrust = " << max_.thrust);

  command_msg_.normalized_roll = 0;
  command_msg_.normalized_pitch = 0;
  command_msg_.normalized_yaw = 0;
  command_msg_.normalized_throttle = 0;

  current_yaw_vel_ = 0;

  namespace_ = nh_.getNamespace();
  autopilot_command_sub_ = nh_.subscribe(autopilot_command_topic_, 10, &Joy::APCommandCallback, this);
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
  fly_mav_ = true;
  buttons_.mode.prev_value=1;
}

void Joy::StopMav() {

  command_msg_.normalized_roll = 0;
  command_msg_.normalized_pitch = 0;
  command_msg_.normalized_yaw = 0;
  command_msg_.normalized_throttle = 0;
}

void Joy::APCommandCallback(const fcu_common::CommandConstPtr &msg)
{
    autopilot_command_ = *msg;
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  static int mode = -1;
//  if(fly_mav_)
//  {
    current_joy_ = *msg;

    command_msg_.normalized_throttle = 0.5*(msg->axes[axes_.thrust] * axes_.thrust_direction + 1.0);
    command_msg_.normalized_roll = -1.0*msg->axes[axes_.roll] * axes_.roll_direction;
    command_msg_.normalized_pitch = -1.0*msg->axes[axes_.pitch] * axes_.pitch_direction;
    command_msg_.normalized_yaw = msg->axes[axes_.yaw] * axes_.yaw_direction;

    extended_command_msg_.x = command_msg_.normalized_roll;
    extended_command_msg_.y = command_msg_.normalized_pitch;
    extended_command_msg_.z = command_msg_.normalized_yaw;
    extended_command_msg_.F = command_msg_.normalized_throttle;

    switch(extended_command_msg_.mode)
    {
    case fcu_common::ExtendedCommand::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
      extended_command_msg_.x *= max_.roll_rate;
      extended_command_msg_.y *= max_.pitch_rate;
      extended_command_msg_.z *= max_.yaw_rate;
      break;
    case fcu_common::ExtendedCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE:
      extended_command_msg_.x *= max_.roll;
      extended_command_msg_.y *= max_.pitch;
      extended_command_msg_.z *= max_.yaw_rate;
      break;
    case fcu_common::ExtendedCommand::MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
      extended_command_msg_.x *= max_.roll;
      extended_command_msg_.y *= max_.pitch;
      extended_command_msg_.z *= max_.yaw_rate;
      extended_command_msg_.F *= max_.altitude;
      break;
    }

    if(msg->buttons[1] == 1)
    {
        command_msg_ = autopilot_command_;
        //command_msg_.normalized_throttle = 0.5*(msg->axes[axes_.thrust] + 1.0);
        //command_msg_.normalized_roll = -1.0*msg->axes[axes_.roll] * max_.aileron * axes_.roll_direction;
        //command_msg_.normalized_pitch = -1.0*msg->axes[axes_.pitch] * max_.elevator * axes_.pitch_direction;
        //command_msg_.normalized_yaw = msg->axes[axes_.yaw] * max_.rudder * axes_.yaw_direction;
    }

//  }
//  else
//  {
//    StopMav();
//  }
//  if(msg->buttons[buttons_.fly.index]==0 && buttons_.fly.prev_value==1){ // button release
//    fly_mav_ = !fly_mav_;
//  }
//  buttons_.fly.prev_value = msg->buttons[buttons_.fly.index];

  if(msg->buttons[buttons_.mode.index]==0 && buttons_.mode.prev_value==1){ // button release
    mode = (mode+1)%4;
    if(mode == 0)
    {
      ROS_INFO("Rate Mode");
      extended_command_msg_.mode = fcu_common::ExtendedCommand::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
    }
    else if(mode == 1)
    {
      ROS_INFO("Angle Mode");
      extended_command_msg_.mode = fcu_common::ExtendedCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    }
    else if (mode == 2)
    {
      ROS_INFO("Altitude Mode");
      extended_command_msg_.mode = fcu_common::ExtendedCommand::MODE_ROLL_PITCH_YAWRATE_ALTITUDE;
    }
    else if (mode == 3)
    {
      ROS_INFO("Passthrough");
      extended_command_msg_.mode = fcu_common::ExtendedCommand::MODE_PASS_THROUGH;
    }
  }
  buttons_.mode.prev_value = msg->buttons[buttons_.mode.index];


  Publish();
}

void Joy::Publish() {
  command_pub_.publish(command_msg_);
  extended_command_pub_.publish(extended_command_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fcu_common_joy");
  Joy joy;

  ros::spin();

  return 0;
}
