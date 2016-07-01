/**
 * \file fcu_io.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/serial_exception.h>
#include <string>
#include <stdint.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "fcu_io.h"

namespace fcu_io
{

fcuIO::fcuIO()
{
  ros::NodeHandle nh;

  command_sub_ = nh.subscribe("extended_command", 1, &fcuIO::commandCallback, this);

  unsaved_params_pub_ = nh.advertise<std_msgs::Bool>("unsaved_params", 1, true);
  imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
  imu_temp_pub_ = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 1);
  servo_output_raw_pub_ = nh.advertise<fcu_common::ServoOutputRaw>("servo_output_raw", 1);
  rc_raw_pub_ = nh.advertise<fcu_common::ServoOutputRaw>("rc_raw", 1);
  diff_pressure_pub_ = nh.advertise<sensor_msgs::FluidPressure>("diff_pressure", 1);
  temperature_pub_ = nh.advertise<sensor_msgs::Temperature>("temperature", 1);
  baro_pub_ = nh.advertise<std_msgs::Float32>("baro/alt", 1);

  param_get_srv_ = nh.advertiseService("param_get", &fcuIO::paramGetSrvCallback, this);
  param_set_srv_ = nh.advertiseService("param_set", &fcuIO::paramSetSrvCallback, this);
  param_write_srv_ = nh.advertiseService("param_write", &fcuIO::paramWriteSrvCallback, this);
  imu_calibrate_srv_ = nh.advertiseService("calibrate_imu", &fcuIO::calibrateIMUCallback, this);

  ros::NodeHandle nh_private("~");
  std::string port = nh_private.param<std::string>("port", "/dev/ttyUSB0");
  int baud_rate = nh_private.param<int>("baud_rate", 115200);


  try
  {
    mavrosflight_ = new mavrosflight::MavROSflight(port, baud_rate);
  }
  catch (mavrosflight::SerialException e)
  {
    ROS_FATAL("%s", e.what());
    ros::shutdown();
  }

  mavrosflight_->serial.register_mavlink_listener(this);
  mavrosflight_->param.register_param_listener(this);

  std_msgs::Bool unsaved_msg;
  unsaved_msg.data = false;
  unsaved_params_pub_.publish(unsaved_msg);
}

fcuIO::~fcuIO()
{
  delete mavrosflight_;
}

void fcuIO::handle_mavlink_message(const mavlink_message_t &msg)
{
  switch (msg.msgid)
  {
  case MAVLINK_MSG_ID_HEARTBEAT:
    handle_heartbeat_msg();
    break;
  case MAVLINK_MSG_ID_SMALL_IMU:
    handle_small_imu_msg(msg);
    break;
  case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
    handle_servo_output_raw_msg(msg);
    break;
  case MAVLINK_MSG_ID_RC_CHANNELS:
    handle_rc_channels_raw_msg(msg);
    break;
  case MAVLINK_MSG_ID_DIFF_PRESSURE:
    handle_diff_pressure_msg(msg);
    break;
  case MAVLINK_MSG_ID_NAMED_VALUE_INT:
    handle_named_value_int_msg(msg);
    break;
  case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
    handle_named_value_float_msg(msg);
    break;
  case MAVLINK_MSG_ID_SMALL_BARO:
    handle_small_baro_msg(msg);
    break;
  case MAVLINK_MSG_ID_TIMESYNC: // <-- This is handled by the time_manager
    break;
  case MAVLINK_MSG_ID_PARAM_VALUE: // <-- This is handled by the param_manager
    break;
  default:
    ROS_ERROR("fcu_io:Got unsupported message ID %d", msg.msgid);
    break;
  }
}

void fcuIO::on_new_param_received(std::string name, double value)
{
  ROS_INFO("Got parameter %s with value %g", name.c_str(), value);
}

void fcuIO::on_param_value_updated(std::string name, double value)
{
  ROS_INFO("Parameter %s has new value %g", name.c_str(), value);
}

void fcuIO::on_params_saved_change(bool unsaved_changes)
{
  std_msgs::Bool msg;
  msg.data = unsaved_changes;
  unsaved_params_pub_.publish(msg);

  if (unsaved_changes)
  {
    ROS_WARN("There are unsaved changes to onboard parameters");
  }
  else
  {
    ROS_INFO("Onboard parameters have been saved");
  }
}

void fcuIO::handle_heartbeat_msg()
{
  ROS_INFO_ONCE("Got HEARTBEAT, connected.");
}

void fcuIO::handle_small_imu_msg(const mavlink_message_t &msg)
{
  mavlink_small_imu_t imu;
  mavlink_msg_small_imu_decode(&msg, &imu);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = mavrosflight_->time.get_ros_time_us(imu.time_boot_us);

  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = imu_msg.header.stamp;

  static int send_throttle = 0;

  // This is so we can eventually make calibrating the IMU an external service
  if (!imu_.calibrated)
  {
    if(imu_.calibrate(imu))
    {
      ROS_INFO("accel parameters found\n xm = %f, ym = %f, zm = %f xb = %f yb = %f, zb = %f",
               imu_.xm(),imu_.ym(),imu_.zm(),imu_.xb(),imu_.yb(),imu_.zb());
        // calibration is done, send params to the param server and save them
        /// DOING IT THIS WAY ENSURES THAT FCU_IO WILL CRASH -- I THINK WE ARE OVERFLOWING A BUFFER SOMEWHERE
        mavrosflight_->param.set_param_value("ACC_X_TEMP_COMP", 1000.0*imu_.xm());
        mavrosflight_->param.set_param_value("ACC_Y_TEMP_COMP", 1000.0*imu_.ym());
        mavrosflight_->param.set_param_value("ACC_Z_TEMP_COMP", 1000.0*imu_.zm());
        mavrosflight_->param.set_param_value("ACC_X_BIAS", 1000.0*imu_.xb());
        mavrosflight_->param.set_param_value("ACC_Y_BIAS", 1000.0*imu_.yb());
        mavrosflight_->param.set_param_value("ACC_Z_BIAS", 1000.0*imu_.zb());
        mavrosflight_->param.write_params();
    }
  }


  bool valid = imu_.correct(imu,
                            &imu_msg.linear_acceleration.x,
                            &imu_msg.linear_acceleration.y,
                            &imu_msg.linear_acceleration.z,
                            &imu_msg.angular_velocity.x,
                            &imu_msg.angular_velocity.y,
                            &imu_msg.angular_velocity.z,
                            &temp_msg.temperature);

  if (valid)
  {
    imu_pub_.publish(imu_msg);
    imu_temp_pub_.publish(temp_msg);
  }
}

void fcuIO::handle_servo_output_raw_msg(const mavlink_message_t &msg)
{
  mavlink_servo_output_raw_t servo;
  mavlink_msg_servo_output_raw_decode(&msg, &servo);

  fcu_common::ServoOutputRaw out_msg;
  out_msg.header.stamp = mavrosflight_->time.get_ros_time_us(servo.time_usec);
  out_msg.port = servo.port;

  out_msg.values[0] = servo.servo1_raw;
  out_msg.values[1] = servo.servo2_raw;
  out_msg.values[2] = servo.servo3_raw;
  out_msg.values[3] = servo.servo4_raw;
  out_msg.values[4] = servo.servo5_raw;
  out_msg.values[5] = servo.servo6_raw;
  out_msg.values[6] = servo.servo7_raw;
  out_msg.values[7] = servo.servo8_raw;

  servo_output_raw_pub_.publish(out_msg);
}

void fcuIO::handle_rc_channels_raw_msg(const mavlink_message_t &msg)
{
  mavlink_rc_channels_raw_t rc;
  mavlink_msg_rc_channels_raw_decode(&msg, &rc);

  fcu_common::ServoOutputRaw out_msg;
  out_msg.header.stamp = mavrosflight_->time.get_ros_time_ms(rc.time_boot_ms);
  out_msg.port = rc.port;

  out_msg.values[0] = rc.chan1_raw;
  out_msg.values[1] = rc.chan2_raw;
  out_msg.values[2] = rc.chan3_raw;
  out_msg.values[3] = rc.chan4_raw;
  out_msg.values[4] = rc.chan5_raw;
  out_msg.values[5] = rc.chan6_raw;
  out_msg.values[6] = rc.chan7_raw;
  out_msg.values[7] = rc.chan8_raw;

  rc_raw_pub_.publish(out_msg);
}

void fcuIO::handle_diff_pressure_msg(const mavlink_message_t &msg)
{
  mavlink_diff_pressure_t diff;
  mavlink_msg_diff_pressure_decode(&msg, &diff);

  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = ros::Time::now(); //! \todo time synchronization

  sensor_msgs::FluidPressure pressure_msg;
  pressure_msg.header.stamp = ros::Time::now(); //! \todo time synchronization

  bool valid = diff_pressure_.correct(diff,
                                      &pressure_msg.fluid_pressure,
                                      &temp_msg.temperature);

  if (valid)
  {
    temperature_pub_.publish(temp_msg);
    diff_pressure_pub_.publish(pressure_msg);
  }
}

void fcuIO::handle_named_value_int_msg(const mavlink_message_t &msg)
{
  mavlink_named_value_int_t val;
  mavlink_msg_named_value_int_decode(&msg, &val);

  std::string name(val.name);
  if (named_value_int_pubs_.find(name) == named_value_int_pubs_.end())
  {
    ros::NodeHandle nh;
    named_value_int_pubs_[name] = nh.advertise<std_msgs::Int32>("named_value/int/" + name, 1);
  }

  std_msgs::Int32 out_msg;
  out_msg.data = val.value;

  named_value_int_pubs_[name].publish(out_msg);
}

void fcuIO::handle_named_value_float_msg(const mavlink_message_t &msg)
{
  mavlink_named_value_float_t val;
  mavlink_msg_named_value_float_decode(&msg, &val);

  std::string name(val.name);
  if (named_value_float_pubs_.find(name) == named_value_float_pubs_.end())
  {
    ros::NodeHandle nh;
    named_value_float_pubs_[name] = nh.advertise<std_msgs::Float32>("named_value/float/" + name, 1);
  }

  std_msgs::Float32 out_msg;
  out_msg.data = val.value;

  named_value_float_pubs_[name].publish(out_msg);
}

void fcuIO::handle_small_baro_msg(const mavlink_message_t &msg)
{
  mavlink_small_baro_t baro;
  mavlink_msg_small_baro_decode(&msg, &baro);

  double alt = 0;

  if (baro_.correct(baro, &alt))
  {
    std_msgs::Float32 alt_msg;
    alt_msg.data = alt;
    baro_pub_.publish(alt_msg);
  }
}

void fcuIO::commandCallback(fcu_common::ExtendedCommand::ConstPtr msg)
{
  //! \todo these are hard-coded to match right now; may want to replace with something more robust
  OFFBOARD_CONTROL_MODE mode = (OFFBOARD_CONTROL_MODE) msg->mode;
  OFFBOARD_CONTROL_IGNORE ignore = (OFFBOARD_CONTROL_IGNORE) msg->ignore;

  int v1 = (int)(msg->value1 * 1000);
  int v2 = (int)(msg->value2 * 1000);
  int v3 = (int)(msg->value3 * 1000);
  int v4 = (int)(msg->value4 * 1000);

  switch (mode)
  {
  case MODE_PASS_THROUGH:
    v1 = saturate(v1, -1000, 1000);
    v2 = saturate(v2, -1000, 1000);
    v3 = saturate(v3, -1000, 1000);
    v4 = saturate(v4, -1000, 1000);
    break;
  case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
  case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
    v4 = saturate(v4, 0, 1000);
    break;
  case MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
    break;
  }

  mavlink_message_t mavlink_msg;
  mavlink_msg_offboard_control_pack(1, 50, &mavlink_msg,
                                    mode, ignore, (int16_t)v1, (int16_t)v2, (int16_t)v3, (int16_t)v4);
  mavrosflight_->serial.send_message(mavlink_msg);
}

bool fcuIO::paramGetSrvCallback(fcu_io::ParamGet::Request &req, fcu_io::ParamGet::Response &res)
{
  res.exists = mavrosflight_->param.get_param_value(req.name, &res.value);
  return true;
}

bool fcuIO::paramSetSrvCallback(fcu_io::ParamSet::Request &req, fcu_io::ParamSet::Response &res)
{
  res.exists = mavrosflight_->param.set_param_value(req.name, req.value);
  return true;
}

bool fcuIO::paramWriteSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = mavrosflight_->param.write_params();
  if (!res.success)
  {
    res.message = "Request rejected: write already in progress";
  }

  return true;
}

bool fcuIO::calibrateIMUCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  // tell the IMU to calibrate itself
  imu_.calibrated = false;
  res.success = true;
}

} // namespace fcu_io
