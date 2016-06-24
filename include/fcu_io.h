/**
 * \file fcu_io.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef FCU_IO_MAVROSFLIGHT_ROS_H
#define FCU_IO_MAVROSFLIGHT_ROS_H

#include <map>
#include <string>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Empty.h>

#include <fcu_common/ExtendedCommand.h>
#include <fcu_common/ServoOutputRaw.h>

#include <fcu_io/ParamRequestList.h>
#include <fcu_io/ParamRequestRead.h>
#include <fcu_io/ParamSet.h>

#include <mavrosflight/mavrosflight.h>

namespace fcu_io
{

class fcuIO : public mavrosflight::MavlinkListenerInterface
{
public:
  fcuIO();
  ~fcuIO();

  virtual void handle_mavlink_message(const mavlink_message_t &msg);

private:

  // handle mavlink messages
  void handle_heartbeat_msg();
  void handle_param_value_msg(const mavlink_message_t &msg);
  void handle_small_imu_msg(const mavlink_message_t &msg);
  void handle_servo_output_raw_msg(const mavlink_message_t &msg);
  void handle_rc_channels_raw_msg(const mavlink_message_t &msg);
  void handle_diff_pressure_msg(const mavlink_message_t &msg);
  void handle_command_ack_msg(const mavlink_message_t &msg);
  void handle_named_value_int_msg(const mavlink_message_t &msg);
  void handle_named_value_float_msg(const mavlink_message_t &msg);

  // ROS message callbacks
  void commandCallback(fcu_common::ExtendedCommand::ConstPtr msg);

  // ROS service callbacks
  bool paramRequestListSrvCallback(fcu_io::ParamRequestList::Request &req, fcu_io::ParamRequestList::Response &res);
  bool paramRequestReadSrvCallback(fcu_io::ParamRequestRead::Request &req, fcu_io::ParamRequestRead::Response &res);
  bool paramSetSrvCallback(fcu_io::ParamSet::Request &req, fcu_io::ParamSet::Response &res);
  bool paramWriteSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  ros::Subscriber command_sub_;

  ros::Publisher imu_pub_;
  ros::Publisher servo_output_raw_pub_;
  ros::Publisher rc_raw_pub_;
  ros::Publisher diff_pressure_pub_;
  ros::Publisher temperature_pub_;
  std::map<std::string, ros::Publisher> named_value_int_pubs_;
  std::map<std::string, ros::Publisher> named_value_float_pubs_;

  ros::ServiceServer param_request_list_srv_;
  ros::ServiceServer param_request_read_srv_;
  ros::ServiceServer param_set_srv_;
  ros::ServiceServer param_write_srv_;

  mavrosflight::MavROSflight* mavrosflight_;
};

} // namespace fcu_io

#endif // FCU_IO_MAVROSFLIGHT_ROS_H
