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

  pnh.param<int>("x_axis", axes_.roll, 2);
  pnh.param<int>("y_axis", axes_.pitch, 3);
  pnh.param<int>("axis_thrust_", axes_.thrust, 1);
  pnh.param<int>("yaw_axis", axes_.yaw, 0);

  pnh.param<int>("x_sign", axes_.roll_direction, -1);
  pnh.param<int>("y_sign", axes_.pitch_direction, -1);
  pnh.param<int>("axis_direction_thrust", axes_.thrust_direction, 1);
  pnh.param<int>("yaw_sign", axes_.yaw_direction, -1);

  pnh.param<double>("max_roll", max_.roll, 25.0 * M_PI / 180.0);  // [rad]
  pnh.param<double>("max_pitch", max_.pitch, 25.0 * M_PI / 180.0);  // [rad]
  pnh.param<double>("max_yaw_rate", max_.rate_yaw, 180.0 * M_PI / 180.0);  // [rad/s]
  pnh.param<double>("max_thrust", max_.thrust, 74.676);  // [N]
  pnh.param<double>("mass", mass_, 3.81);  // [N]

  pnh.param<double>("max_aileron", max_.aileron, 15.0*M_PI/180.0);
  pnh.param<double>("max_elevator", max_.elevator, 25.0*M_PI/180.0);
  pnh.param<double>("max_rudder", max_.rudder, 15.0*M_PI/180.0);

  pnh.param<int>("button_takeoff_", buttons_.fly.index, 0);

  command_pub_ = nh_.advertise<fcu_common::Command>(command_topic_,10);

//  ROS_ERROR_STREAM("mass = " << mass_ <<" max_thrust = " << max_.thrust);

  command_msg_.normalized_roll = 0;
  command_msg_.normalized_pitch = 0;
  command_msg_.normalized_yaw = 0;
  command_msg_.normalized_throttle = 0;

  current_yaw_vel_ = 0;

  namespace_ = nh_.getNamespace();
  autopilot_command_sub_ = nh_.subscribe(autopilot_command_topic_, 10, &Joy::APCommandCallback, this);
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
  fly_mav_ = false;
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
  if(fly_mav_){
    current_joy_ = *msg;

    command_msg_.normalized_throttle = 0.5*(msg->axes[axes_.thrust] + 1.0);
    command_msg_.normalized_roll = -1.0*msg->axes[axes_.roll] * max_.aileron * axes_.roll_direction;
    command_msg_.normalized_pitch = -1.0*msg->axes[axes_.pitch] * max_.elevator * axes_.pitch_direction;
    command_msg_.normalized_yaw = msg->axes[axes_.yaw] * max_.rudder * axes_.yaw_direction;

    if(msg->buttons[1] == 1)
    {
        command_msg_ = autopilot_command_;
        //command_msg_.normalized_throttle = 0.5*(msg->axes[axes_.thrust] + 1.0);
        //command_msg_.normalized_roll = -1.0*msg->axes[axes_.roll] * max_.aileron * axes_.roll_direction;
        //command_msg_.normalized_pitch = -1.0*msg->axes[axes_.pitch] * max_.elevator * axes_.pitch_direction;
        //command_msg_.normalized_yaw = msg->axes[axes_.yaw] * max_.rudder * axes_.yaw_direction;
    }

  } else{
    StopMav();
  }
  if(msg->buttons[buttons_.fly.index]==0 && buttons_.fly.prev_value==1){ // button release
    fly_mav_ = !fly_mav_;
  }
  buttons_.fly.prev_value = msg->buttons[buttons_.fly.index];

  Publish();
}

void Joy::Publish() {
  command_pub_.publish(command_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fcu_common_joy");
  Joy joy;

  ros::spin();

  return 0;
}
