/**
 * \file fcu_io.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef FCU_IO_MAVROSFLIGHT_ROS_H
#define FCU_IO_MAVROSFLIGHT_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

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

  bool paramRequestListSrvCallback(fcu_io::ParamRequestList::Request &req, fcu_io::ParamRequestList::Response &res);
  bool paramRequestReadSrvCallback(fcu_io::ParamRequestRead::Request &req, fcu_io::ParamRequestRead::Response &res);
  bool paramSetSrvCallback(fcu_io::ParamSet::Request &req, fcu_io::ParamSet::Response &res);

  ros::Publisher imu_pub_;

  ros::ServiceServer param_request_list_srv_;
  ros::ServiceServer param_request_read_srv_;
  ros::ServiceServer param_set_srv_;

  mavrosflight::MavROSflight* mavrosflight_;
};

} // namespace fcu_io

#endif // FCU_IO_MAVROSFLIGHT_ROS_H
