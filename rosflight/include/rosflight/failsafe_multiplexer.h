//
// Created by trey on 6/28/20.
//

#ifndef URBAN_TAKEOFF_FAILSAFE_MULTIPLEXER_H
#define URBAN_TAKEOFF_FAILSAFE_MULTIPLEXER_H

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include "rosflight_msgs/Status.h"
#include "rosflight_msgs/Command.h"

namespace failsafe_multiplexer
{
class FailsafeMultiplexer
{
public:
  FailsafeMultiplexer();

protected:
  void status_cb(const rosflight_msgs::Status &status);
  void normal_command_cb(const rosflight_msgs::Command &command);
  void failsafe_command_cb(const rosflight_msgs::Command &command);
  bool failsafe_force_service_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
private:
  ros::NodeHandle node_handle_;
  ros::Publisher command_pub_;
  ros::Subscriber status_sub_;
  ros::Subscriber failsafe_command_sub_;
  ros::Subscriber normal_command_sub_;
  ros::ServiceServer set_force_failsafe_service_server_;

  inline bool do_failsafe(){return in_failsafe_ || force_failsafe_;}
  bool in_failsafe_{ false };
  bool force_failsafe_{false};

};
} // namespace failsafe_multiplexer
#endif //URBAN_TAKEOFF_FAILSAFE_MULTIPLEXER_H
