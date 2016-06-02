/**
 * \file fcu_io.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef FCU_IO_MAVROSFLIGHT_ROS_H
#define FCU_IO_MAVROSFLIGHT_ROS_H

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

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

  void paramCallback(char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN], float param_value, MAV_PARAM_TYPE param_type);
  void heartbeatCallback();
  void imuCallback(double xacc, double yacc, double zacc, double xgyro, double ygyro, double zgyro);
  void servoOutputRawCallback(uint32_t time_usec, uint8_t port, uint16_t values[8]);
  void commandAckCallback(uint16_t command, uint8_t result);

  bool paramRequestListSrvCallback(fcu_io::ParamRequestList::Request &req, fcu_io::ParamRequestList::Response &res);
  bool paramRequestReadSrvCallback(fcu_io::ParamRequestRead::Request &req, fcu_io::ParamRequestRead::Response &res);
  bool paramSetSrvCallback(fcu_io::ParamSet::Request &req, fcu_io::ParamSet::Response &res);
  bool paramWriteSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

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
