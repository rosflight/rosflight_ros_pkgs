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

#include <rosflight_utils/joy.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_srvs/Empty.h>

Joy::Joy()
{
  ros::NodeHandle pnh("~");
  ros::NodeHandle namespace_nh(ros::this_node::getNamespace());

  pnh.param<std::string>("command_topic", command_topic_, "command");
  pnh.param<std::string>("autopilot_command_topic", autopilot_command_topic_, "autopilot_command");

  // Get global parameters
  double max_thrust;
  namespace_nh.param<double>("mass", mass_, 3.61);
  namespace_nh.param<double>("max_F", max_thrust, 64.50);
  namespace_nh.param<std::string>("mav_name", mav_name_,"shredder");
  equilibrium_thrust_ = (mass_*9.80665) / max_thrust;

  // Get Parameters from joystick configuration yaml
  pnh.param<std::string>("gazebo_namespace", gazebo_ns_, "");
  pnh.param<int>("x_axis", axes_.x, 1);
  pnh.param<int>("y_axis", axes_.y, 2);
  pnh.param<int>("F_axis", axes_.F, 0);
  pnh.param<int>("z_axis", axes_.z, 4);

  pnh.param<int>("x_sign", axes_.x_direction, 1);
  pnh.param<int>("y_sign", axes_.y_direction, 1);
  pnh.param<int>("F_sign", axes_.F_direction, -1);
  pnh.param<int>("z_sign", axes_.z_direction, 1);

  pnh.param<double>("max_aileron", max_.aileron, 15.0 * M_PI / 180.0);
  pnh.param<double>("max_elevator", max_.elevator, 25.0 * M_PI / 180.0);
  pnh.param<double>("max_rudder", max_.rudder, 15.0 * M_PI / 180.0);

  pnh.param<double>("max_roll_rate", max_.roll_rate, 360.0 * M_PI / 180.0);
  pnh.param<double>("max_pitch_rate", max_.pitch_rate, 360.0 * M_PI / 180.0);
  pnh.param<double>("max_yaw_rate", max_.yaw_rate, 360.0 * M_PI / 180.0);

  pnh.param<double>("max_roll_angle", max_.roll, 45.0 * M_PI / 180.0);
  pnh.param<double>("max_pitch_angle", max_.pitch, 45.0 * M_PI / 180.0);

  pnh.param<double>("max_xvel", max_.xvel, 1.5);
  pnh.param<double>("max_yvel", max_.yvel, 1.5);
  pnh.param<double>("max_zvel", max_.zvel, 1.5);
  command_msg_.mode = pnh.param<int>("control_mode", (int)rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE);

  pnh.param<double>("reset_pos_x", reset_pose_.position.x, 0.0);
  pnh.param<double>("reset_pos_y", reset_pose_.position.y, 0.0);
  pnh.param<double>("reset_pos_z", reset_pose_.position.z, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.x, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.y, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.z, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.w, 0.0);
  pnh.param<double>("reset_linear_twist_x", reset_twist_.linear.x, 0.0);
  pnh.param<double>("reset_linear_twist_y", reset_twist_.linear.y, 0.0);
  pnh.param<double>("reset_linear_twist_z", reset_twist_.linear.z, 0.0);
  pnh.param<double>("reset_angular_twist_x", reset_twist_.angular.x, 0.0);
  pnh.param<double>("reset_angular_twist_x", reset_twist_.angular.y, 0.0);
  pnh.param<double>("reset_angular_twist_x", reset_twist_.angular.z, 0.0);

  // Sets which buttons are tied to which commands
  pnh.param<int>("button_takeoff", buttons_.fly.index, 0);
  pnh.param<int>("button_mode", buttons_.mode.index, 1);
  pnh.param<int>("button_reset", buttons_.reset.index, 9);
  pnh.param<int>("button_pause", buttons_.pause.index, 8);
  pnh.param<int>("button_override", buttons_.override.index, 8);

  command_pub_ = nh_.advertise<rosflight_msgs::Command>(command_topic_, 10);

  command_msg_.x = 0;
  command_msg_.y = 0;
  command_msg_.z = 0;
  command_msg_.F = 0;
  command_msg_.ignore = 0xFF;

  current_yaw_vel_ = 0;

  namespace_ = nh_.getNamespace();
  autopilot_command_sub_ = nh_.subscribe(autopilot_command_topic_, 10, &Joy::APCommandCallback, this);
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
  buttons_.mode.prev_value = 0;
  buttons_.reset.prev_value = 0;

  current_altitude_setpoint_ = current_x_setpoint_ = current_y_setpoint_ = 0.0;
}

void Joy::StopMav()
{
  command_msg_.x = 0;
  command_msg_.y = 0;
  command_msg_.z = 0;
  command_msg_.F = 0;
}

/* Resets the mav back to origin */
void Joy::ResetMav()
{
  ROS_INFO("Mav position reset.");
  ros::NodeHandle n;

  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = (std::string)mav_name_;
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = reset_pose_;
  modelstate.twist = reset_twist_;

  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);
}

// Pauses the gazebo physics and time
void Joy::PauseSimulation()
{
  ROS_INFO("Simulation paused.");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  std_srvs::Empty pauseSim;
  client.call(pauseSim);
}

// Resumes the gazebo physics and time
void Joy::ResumeSimulation()
{
  ROS_INFO("Simulation resumed.");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  std_srvs::Empty resumeSim;
  client.call(resumeSim);
  last_time_ = ros::Time::now().toSec();
}

void Joy::APCommandCallback(const rosflight_msgs::CommandConstPtr &msg)
{
  autopilot_command_ = *msg;
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr &msg)
{
  double dt = ros::Time::now().toSec() - last_time_;
  last_time_ = ros::Time::now().toSec();

  current_joy_ = *msg;

  // Perform button actions
  if (msg->buttons[buttons_.override.index] == 0 && buttons_.override.prev_value == 1)
  {
    override_autopilot_ = true;
  }
  buttons_.override.prev_value = msg->buttons[buttons_.override.index];

  // Resets the mav to the origin
  if (msg->buttons[buttons_.reset.index] == 0 && buttons_.reset.prev_value == 1)  // button release
  {
    ResetMav();
  }
  buttons_.reset.prev_value = msg->buttons[buttons_.reset.index];

  // Pauses/Unpauses the simulation
  if (msg->buttons[buttons_.pause.index] == 0 && buttons_.pause.prev_value == 1)  // button release
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
    if (command_msg_.mode == rosflight_msgs::Command::MODE_PASS_THROUGH)
    {
      ROS_INFO("Passthrough");
    }
    else if (command_msg_.mode == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
    {
      ROS_INFO("Angle Mode");
    }
    else if (command_msg_.mode == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_ALTITUDE)
    {
      ROS_INFO("Altitude Mode");
    }
    else if (command_msg_.mode == rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE)
    {
      ROS_INFO("Velocity Mode");
    }
    else if (command_msg_.mode == rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE)
    {
      ROS_INFO("Position Mode");
    }
    else
    {
      ROS_INFO("Passthrough");
    }
  }
  buttons_.mode.prev_value = msg->buttons[buttons_.mode.index];

  // calculate the output command from the joysticks
  if(override_autopilot_)
  {
    command_msg_.F = msg->axes[axes_.F] * axes_.F_direction;
    command_msg_.x = msg->axes[axes_.x] * axes_.x_direction;
    command_msg_.y = msg->axes[axes_.y] * axes_.y_direction;
    command_msg_.z = msg->axes[axes_.z] * axes_.z_direction;

    switch (command_msg_.mode)
    {
    case rosflight_msgs::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
      command_msg_.x *= max_.roll_rate;
      command_msg_.y *= max_.pitch_rate;
      command_msg_.z *= max_.yaw_rate;
      if (command_msg_.F > 0.0)
      {
        command_msg_.F = equilibrium_thrust_ + (1.0 - equilibrium_thrust_) * command_msg_.F;
      }
      else
      {
        command_msg_.F = equilibrium_thrust_ + (equilibrium_thrust_) * command_msg_.F;
      }
      break;

    case rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE:
      command_msg_.x *= max_.roll;
      command_msg_.y *= max_.pitch;
      command_msg_.z *= max_.yaw_rate;
      if (command_msg_.F > 0.0)
      {
        command_msg_.F = equilibrium_thrust_ + (1.0 - equilibrium_thrust_) * command_msg_.F;
      }
      else
      {
        command_msg_.F = equilibrium_thrust_ + (equilibrium_thrust_) * command_msg_.F;
      }
      break;

    case rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
      command_msg_.x *= max_.roll;
      command_msg_.y *= max_.pitch;
      command_msg_.z *= max_.yaw_rate;
      // Integrate altitude
      current_altitude_setpoint_ -= dt * max_.zvel * command_msg_.F;
      command_msg_.F = current_altitude_setpoint_;
      break;

    case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
    {
      // Remember that roll affects y velocity and pitch affects -x velocity
      double original_x = command_msg_.x;
      command_msg_.x = max_.xvel * -1.0 * command_msg_.y;
      command_msg_.y = max_.yvel * original_x;
      command_msg_.z *= max_.yaw_rate;
      // Integrate altitude
      current_altitude_setpoint_ -= dt * max_.zvel * command_msg_.F;
      command_msg_.F = current_altitude_setpoint_;
      break;
    }

    case rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE:
      // Integrate all axes
      // (Remember that roll affects y velocity and pitch affects -x velocity)
      current_x_setpoint_ -= dt * max_.xvel * command_msg_.y;
      command_msg_.x = current_x_setpoint_;

      current_y_setpoint_ += dt * max_.yvel * command_msg_.x;
      command_msg_.y = current_y_setpoint_;

      current_yaw_setpoint_ += dt * max_.yaw_rate * command_msg_.z;
      current_yaw_setpoint_ = fmod(current_yaw_setpoint_, (2.0 * M_PI));

      current_altitude_setpoint_ -= dt * max_.zvel * command_msg_.z;
      command_msg_.z = current_altitude_setpoint_;
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
  command_pub_.publish(command_msg_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosflight_utils_joy");
  Joy joy;

  ros::spin();

  return 0;
}
