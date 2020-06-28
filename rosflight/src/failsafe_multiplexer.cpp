//
// Created by trey on 6/28/20.
//

#include "rosflight/failsafe_multiplexer.h"

namespace failsafe_multiplexer
{
FailsafeMultiplexer::FailsafeMultiplexer()
{
  command_pub_ = node_handle_.advertise<rosflight_msgs::Command>("command", 1);
  status_sub_ = node_handle_.subscribe("status", 1, &FailsafeMultiplexer::status_cb, this);
  normal_command_sub_ = node_handle_.subscribe("normal_command", 1, &FailsafeMultiplexer::normal_command_cb, this);
  failsafe_command_sub_ = node_handle_.subscribe("failsafe_command", 1, &FailsafeMultiplexer::failsafe_command_cb, this);
}

void FailsafeMultiplexer::status_cb(const rosflight_msgs::Status &status)
{
  in_failsafe_ = status.failsafe;
}

void FailsafeMultiplexer::normal_command_cb(const rosflight_msgs::Command &command)
{
  if(!in_failsafe_)
    command_pub_.publish(command);
}

void FailsafeMultiplexer::failsafe_command_cb(const rosflight_msgs::Command &command)
{
  if(in_failsafe_)
    command_pub_.publish(command);
}

bool FailsafeMultiplexer::failsafe_force_service_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  return false;
}
} // failsafe_multiplexer

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "failsafe_multiplexer");
  failsafe_multiplexer::FailsafeMultiplexer node = failsafe_multiplexer::FailsafeMultiplexer();
  ros::spin();
  return 0;
}


