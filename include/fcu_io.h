/**
 * \file fcu_io.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef FCU_IO_MAVROSFLIGHT_ROS_H
#define FCU_IO_MAVROSFLIGHT_ROS_H

#include <map>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

#include <fcu_io/Command.h>
#include <fcu_io/ServoOutputRaw.h>

#include <fcu_io/ParamRequestList.h>
#include <fcu_io/ParamRequestRead.h>
#include <fcu_io/ParamSet.h>

#include <mavrosflight/mavrosflight.h>

namespace fcu_io
{

class fcuIO
{
public:
  fcuIO();
  ~fcuIO();

private:

  // mavrosflight callbacks
  void paramCallback(char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN], float param_value, MAV_PARAM_TYPE param_type);
  void heartbeatCallback();
  void imuCallback(double xacc, double yacc, double zacc, double xgyro, double ygyro, double zgyro);
  void servoOutputRawCallback(uint32_t time_usec, uint8_t port, uint16_t values[8]);
  void commandAckCallback(uint16_t command, uint8_t result);
  void namedValueIntCallback(uint32_t time, std::string name, int32_t value);

  // ROS message callbacks
  void commandCallback(fcu_io::Command::ConstPtr msg);

  // ROS service callbacks
  bool paramRequestListSrvCallback(fcu_io::ParamRequestList::Request &req, fcu_io::ParamRequestList::Response &res);
  bool paramRequestReadSrvCallback(fcu_io::ParamRequestRead::Request &req, fcu_io::ParamRequestRead::Response &res);
  bool paramSetSrvCallback(fcu_io::ParamSet::Request &req, fcu_io::ParamSet::Response &res);
  bool paramWriteSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  ros::Subscriber command_sub_;
  std::map<std::string, ros::Subscriber> named_value_int_pubs_;
  std::map<std::string, ros::Subscriber> named_value_float_pubs_;

  ros::Publisher imu_pub_;
  ros::Publisher servo_output_raw_pub_;

  ros::ServiceServer param_request_list_srv_;
  ros::ServiceServer param_request_read_srv_;
  ros::ServiceServer param_set_srv_;
  ros::ServiceServer param_write_srv_;

  mavrosflight::MavROSflight* mavrosflight_;
};

} // namespace fcu_io

#endif // FCU_IO_MAVROSFLIGHT_ROS_H
