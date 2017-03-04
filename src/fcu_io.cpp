/**
 * \file fcu_io.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/serial_exception.h>
#include <string>
#include <stdint.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>

#include "fcu_io.h"

namespace fcu_io
{
fcuIO::fcuIO()
{
  command_sub_ = nh_.subscribe("command", 1, &fcuIO::commandCallback, this);

  unsaved_params_pub_ = nh_.advertise<std_msgs::Bool>("unsaved_params", 1, true);

  param_get_srv_ = nh_.advertiseService("param_get", &fcuIO::paramGetSrvCallback, this);
  param_set_srv_ = nh_.advertiseService("param_set", &fcuIO::paramSetSrvCallback, this);
  param_write_srv_ = nh_.advertiseService("param_write", &fcuIO::paramWriteSrvCallback, this);
  param_save_to_file_srv_ = nh_.advertiseService("param_save_to_file", &fcuIO::paramSaveToFileCallback, this);
  param_load_from_file_srv_ = nh_.advertiseService("param_load_from_file", &fcuIO::paramLoadFromFileCallback, this);
  imu_calibrate_bias_srv_ = nh_.advertiseService("calibrate_imu_bias", &fcuIO::calibrateImuBiasSrvCallback, this);
  imu_calibrate_temp_srv_ = nh_.advertiseService("calibrate_imu_temp", &fcuIO::calibrateImuTempSrvCallback, this);
  calibrate_rc_srv_ = nh_.advertiseService("calibrate_rc_trim", &fcuIO::calibrateRCTrimSrvCallback, this);
  reboot_srv_ = nh_.advertiseService("reboot", &fcuIO::rebootSrvCallback, this);

  ros::NodeHandle nh_private("~");
  std::string port = nh_private.param<std::string>("port", "/dev/ttyUSB0");
  int baud_rate = nh_private.param<int>("baud_rate", 921600);
  frame_id_ = nh_private.param<std::string>("frame_id", "world");

  ROS_INFO("FCU_IO");
  ROS_INFO("Connecting to %s, at %d baud", port.c_str(), baud_rate);

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

  // request the param list
  mavrosflight_->param.request_params();
  param_timer_ = nh_.createTimer(ros::Duration(1.0), &fcuIO::paramTimerCallback, this);

  // initialize latched "unsaved parameters" message value
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
      handle_heartbeat_msg(msg);
      break;
    case MAVLINK_MSG_ID_COMMAND_ACK:
      handle_command_ack_msg(msg);
      break;
    case MAVLINK_MSG_ID_STATUSTEXT:
      handle_statustext_msg(msg);
      break;
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
      handle_attitude_quaternion_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_IMU:
      handle_small_imu_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_MAG:
      handle_small_mag_msg(msg);
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
    case MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT:
      handle_named_command_struct_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_BARO:
      handle_small_baro_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_SONAR:
      handle_small_sonar(msg);
      break;
    default:
      ROS_DEBUG("fcu_io: Got unhandled mavlink message ID %d", msg.msgid);
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

void fcuIO::handle_heartbeat_msg(const mavlink_message_t &msg)
{
  ROS_INFO_ONCE("Got HEARTBEAT, connected.");

  static int prev_armed_state = 0;
  static int prev_control_mode = 0;
  mavlink_heartbeat_t heartbeat;
  mavlink_msg_heartbeat_decode(&msg, &heartbeat);

  // Print if change in armed_state
  if (heartbeat.base_mode != prev_armed_state)
  {
    if (heartbeat.base_mode == MAV_MODE_MANUAL_ARMED)
      ROS_WARN("FCU ARMED");
    else if (heartbeat.base_mode == MAV_MODE_MANUAL_DISARMED)
      ROS_WARN("FCU DISARMED");
    prev_armed_state = heartbeat.base_mode;
  }
  else if (heartbeat.base_mode == MAV_MODE_ENUM_END)
    ROS_ERROR_THROTTLE(600, "FCU FAILSAFE");

  // Print if change in control mode
  if (heartbeat.custom_mode != prev_control_mode)
  {
    std::string mode_string;
    switch (heartbeat.custom_mode)
    {
      case MODE_PASS_THROUGH:
        mode_string = "PASS_THROUGH";
        break;
      case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
        mode_string = "RATE";
        break;
      case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
        mode_string = "ANGLE";
        break;
      case MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
        mode_string = "ALTITUDE";
        break;
      default:
        mode_string = "UNKNOWN";
    }
    ROS_WARN_STREAM("FCU now in " << mode_string << " mode");
    prev_control_mode = heartbeat.custom_mode;
  }
}

void fcuIO::handle_command_ack_msg(const mavlink_message_t &msg)
{
  mavlink_command_ack_t ack;
  mavlink_msg_command_ack_decode(&msg, &ack);

  switch (ack.command)
  {
    case MAV_CMD_PREFLIGHT_CALIBRATION:
      if (ack.result == MAV_RESULT_ACCEPTED)
      {
        ROS_INFO("Sensor calibration Acknowledged!");
      }
      else
      {
        ROS_ERROR("Sensor calibration failed");
      }
      break;
  }
}

void fcuIO::handle_statustext_msg(const mavlink_message_t &msg)
{
  mavlink_statustext_t status;
  mavlink_msg_statustext_decode(&msg, &status);

  // ensure null termination
  char c_str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
  memcpy(c_str, status.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
  c_str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = '\0';

  switch (status.severity)
  {
    case MAV_SEVERITY_EMERGENCY:
    case MAV_SEVERITY_ALERT:
    case MAV_SEVERITY_CRITICAL:
    case MAV_SEVERITY_ERROR:
      ROS_ERROR("[FCU]: %s", c_str);
      break;
    case MAV_SEVERITY_WARNING:
      ROS_WARN("[FCU]: %s", c_str);
      break;
    case MAV_SEVERITY_NOTICE:
    case MAV_SEVERITY_INFO:
      ROS_INFO("[FCU]: %s", c_str);
      break;
    case MAV_SEVERITY_DEBUG:
      ROS_DEBUG("[FCU]: %s", c_str);
      break;
  }
}

void fcuIO::handle_attitude_quaternion_msg(const mavlink_message_t &msg)
{
  mavlink_attitude_quaternion_t attitude;
  mavlink_msg_attitude_quaternion_decode(&msg, &attitude);

  fcu_common::Attitude attitude_msg;
  attitude_msg.header.stamp = mavrosflight_->time.get_ros_time_ms(attitude.time_boot_ms);
  attitude_msg.attitude.w = attitude.q1;
  attitude_msg.attitude.x = attitude.q2;
  attitude_msg.attitude.y = attitude.q3;
  attitude_msg.attitude.z = attitude.q4;
  attitude_msg.angular_velocity.x = attitude.rollspeed;
  attitude_msg.angular_velocity.y = attitude.pitchspeed;
  attitude_msg.angular_velocity.z = attitude.yawspeed;

  geometry_msgs::Vector3Stamped euler_msg;

  tf::Quaternion quat(attitude.q2, attitude.q3, attitude.q4, attitude.q1);
  tf::Matrix3x3(quat).getEulerYPR(euler_msg.vector.z, euler_msg.vector.y, euler_msg.vector.x);

  // save off the quaternion for use with the IMU callback
  tf::quaternionTFToMsg(quat, attitude_quat_);

  if (attitude_pub_.getTopic().empty())
  {
    attitude_pub_ = nh_.advertise<fcu_common::Attitude>("attitude", 1);
  }
  if (euler_pub_.getTopic().empty())
  {
    euler_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("attitude/euler", 1);
  }
  attitude_pub_.publish(attitude_msg);
  euler_pub_.publish(euler_msg);
}

void fcuIO::handle_small_imu_msg(const mavlink_message_t &msg)
{
  mavlink_small_imu_t imu;
  mavlink_msg_small_imu_decode(&msg, &imu);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = mavrosflight_->time.get_ros_time_us(imu.time_boot_us);
  imu_msg.header.frame_id = frame_id_;

  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = imu_msg.header.stamp;
  temp_msg.header.frame_id = frame_id_;

  // This is so we can eventually make calibrating the IMU an external service
  if (imu_.is_calibrating())
  {
    if (imu_.calibrate_temp(imu))
    {
      ROS_INFO("IMU temperature calibration complete:\n xm = %f, ym = %f, zm = %f xb = %f yb = %f, zb = %f", imu_.xm(),
               imu_.ym(), imu_.zm(), imu_.xb(), imu_.yb(), imu_.zb());

      // calibration is done, send params to the param server
      mavrosflight_->param.set_param_value("ACC_X_TEMP_COMP", imu_.xm());
      mavrosflight_->param.set_param_value("ACC_Y_TEMP_COMP", imu_.ym());
      mavrosflight_->param.set_param_value("ACC_Z_TEMP_COMP", imu_.zm());
      mavrosflight_->param.set_param_value("ACC_X_BIAS", imu_.xb());
      mavrosflight_->param.set_param_value("ACC_Y_BIAS", imu_.yb());
      mavrosflight_->param.set_param_value("ACC_Z_BIAS", imu_.zb());

      ROS_WARN("Write params to save new temperature calibration!");
    }
  }

  bool valid = imu_.correct(imu, &imu_msg.linear_acceleration.x, &imu_msg.linear_acceleration.y,
                            &imu_msg.linear_acceleration.z, &imu_msg.angular_velocity.x, &imu_msg.angular_velocity.y,
                            &imu_msg.angular_velocity.z, &temp_msg.temperature);

  imu_msg.orientation = attitude_quat_;

  if (valid)
  {
    if (imu_pub_.getTopic().empty())
    {
      imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
    }
    imu_pub_.publish(imu_msg);

    if (imu_temp_pub_.getTopic().empty())
    {
      imu_temp_pub_ = nh_.advertise<sensor_msgs::Temperature>("imu/temperature", 1);
    }
    imu_temp_pub_.publish(temp_msg);
  }
}

void fcuIO::handle_servo_output_raw_msg(const mavlink_message_t &msg)
{
  mavlink_servo_output_raw_t servo;
  mavlink_msg_servo_output_raw_decode(&msg, &servo);

  fcu_common::OutputRaw out_msg;
  out_msg.header.stamp = mavrosflight_->time.get_ros_time_us(servo.time_usec);

  out_msg.values[0] = servo.servo1_raw;
  out_msg.values[1] = servo.servo2_raw;
  out_msg.values[2] = servo.servo3_raw;
  out_msg.values[3] = servo.servo4_raw;
  out_msg.values[4] = servo.servo5_raw;
  out_msg.values[5] = servo.servo6_raw;
  out_msg.values[6] = servo.servo7_raw;
  out_msg.values[7] = servo.servo8_raw;

  if (servo_output_raw_pub_.getTopic().empty())
  {
    servo_output_raw_pub_ = nh_.advertise<fcu_common::OutputRaw>("output_raw", 1);
  }
  servo_output_raw_pub_.publish(out_msg);
}

void fcuIO::handle_rc_channels_raw_msg(const mavlink_message_t &msg)
{
  mavlink_rc_channels_raw_t rc;
  mavlink_msg_rc_channels_raw_decode(&msg, &rc);

  fcu_common::RCRaw out_msg;
  out_msg.header.stamp = mavrosflight_->time.get_ros_time_ms(rc.time_boot_ms);

  out_msg.values[0] = rc.chan1_raw;
  out_msg.values[1] = rc.chan2_raw;
  out_msg.values[2] = rc.chan3_raw;
  out_msg.values[3] = rc.chan4_raw;
  out_msg.values[4] = rc.chan5_raw;
  out_msg.values[5] = rc.chan6_raw;
  out_msg.values[6] = rc.chan7_raw;
  out_msg.values[7] = rc.chan8_raw;

  if (rc_raw_pub_.getTopic().empty())
  {
    rc_raw_pub_ = nh_.advertise<fcu_common::RCRaw>("rc_raw", 1);
  }
  rc_raw_pub_.publish(out_msg);
}

void fcuIO::handle_diff_pressure_msg(const mavlink_message_t &msg)
{
  mavlink_diff_pressure_t diff;
  mavlink_msg_diff_pressure_decode(&msg, &diff);

  fcu_common::Airspeed airspeed_msg;
  airspeed_msg.header.stamp = ros::Time::now();
  airspeed_msg.velocity = diff.velocity;
  airspeed_msg.differential_pressure = diff.diff_pressure;
  airspeed_msg.temperature = diff.temperature;

  if(calibrate_airspeed_srv_.getService().empty())
  {
    calibrate_airspeed_srv_ = nh_.advertiseService("calibrate_airspeed", &fcuIO::calibrateAirspeedSrvCallback, this);
  }

  if (diff_pressure_pub_.getTopic().empty())
  {
    diff_pressure_pub_ = nh_.advertise<fcu_common::Airspeed>("airspeed", 1);
  }
  diff_pressure_pub_.publish(airspeed_msg);
}

void fcuIO::handle_named_value_int_msg(const mavlink_message_t &msg)
{
  mavlink_named_value_int_t val;
  mavlink_msg_named_value_int_decode(&msg, &val);

  // ensure null termination of name
  char c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN + 1];
  memcpy(c_name, val.name, MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN);
  c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN] = '\0';
  std::string name(c_name);

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

  // ensure null termination of name
  char c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN + 1];
  memcpy(c_name, val.name, MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN);
  c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN] = '\0';
  std::string name(c_name);

  if (named_value_float_pubs_.find(name) == named_value_float_pubs_.end())
  {
    ros::NodeHandle nh;
    named_value_float_pubs_[name] = nh.advertise<std_msgs::Float32>("named_value/float/" + name, 1);
  }

  std_msgs::Float32 out_msg;
  out_msg.data = val.value;

  named_value_float_pubs_[name].publish(out_msg);
}

void fcuIO::handle_named_command_struct_msg(const mavlink_message_t &msg)
{
  mavlink_named_command_struct_t command;
  mavlink_msg_named_command_struct_decode(&msg, &command);

  // ensure null termination of name
  char c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN + 1];
  memcpy(c_name, command.name, MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN);
  c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN] = '\0';
  std::string name(c_name);

  if (named_command_struct_pubs_.find(name) == named_command_struct_pubs_.end())
  {
    ros::NodeHandle nh;
    named_command_struct_pubs_[name] = nh.advertise<fcu_common::Command>("named_value/command_struct/" + name, 1);
  }

  fcu_common::Command command_msg;
  if (command.type == MODE_PASS_THROUGH)
    command_msg.mode = fcu_common::Command::MODE_PASS_THROUGH;
  else if (command.type == MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE)
    command_msg.mode = fcu_common::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  else if (command.type == MODE_ROLL_PITCH_YAWRATE_THROTTLE)
    command_msg.mode = fcu_common::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  else if (command.type == MODE_ROLL_PITCH_YAWRATE_ALTITUDE)
    command_msg.mode = fcu_common::Command::MODE_ROLL_PITCH_YAWRATE_ALTITUDE;

  command_msg.ignore = command.ignore;
  command_msg.x = command.x;
  command_msg.y = command.y;
  command_msg.z = command.z;
  command_msg.F = command.F;
  named_command_struct_pubs_[name].publish(command_msg);
}

void fcuIO::handle_small_baro_msg(const mavlink_message_t &msg)
{
  mavlink_small_baro_t baro;
  mavlink_msg_small_baro_decode(&msg, &baro);

  fcu_common::Barometer baro_msg;
  baro_msg.header.stamp = ros::Time::now();
  baro_msg.altitude = baro.altitude;
  baro_msg.pressure = baro.pressure;
  baro_msg.temperature = baro.temperature;

  // If we are getting barometer messages, then we should publish the barometer calibration service
  if(calibrate_baro_srv_.getService().empty())
  {
    calibrate_baro_srv_ = nh_.advertiseService("calibrate_baro", &fcuIO::calibrateBaroSrvCallback, this);
  }

  if (baro_pub_.getTopic().empty())
  {
    baro_pub_ = nh_.advertise<fcu_common::Barometer>("baro", 1);
  }
  baro_pub_.publish(baro_msg);
}

void fcuIO::handle_small_mag_msg(const mavlink_message_t &msg)
{
  mavlink_small_mag_t mag;
  mavlink_msg_small_mag_decode(&msg, &mag);

  //! \todo calibration, correct units, floating point message type
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.stamp = ros::Time::now();//mavrosflight_->time.get_ros_time_us(mag.time_boot_us);
  mag_msg.header.frame_id = frame_id_;

  if (mag_.is_calibrating())
  {
    if (mag_.calibrate(mag))
    {
      ROS_INFO("Magnetometer calibration complete:\nA = \n%9.5f, %9.5f, %9.5f,\n%9.5f, %9.5f, %9.5f,\n%9.5f, %9.5f, %9.5f\nbx = %9.5f, by = %9.5f, bz = %9.5f",
               mag_.a11(), mag_.a12(), mag_.a13(), mag_.a21(), mag_.a22(), mag_.a23(), mag_.a31(), mag_.a32(), mag_.a33(), mag_.bx(), mag_.by(), mag_.bz());

      // calibration is done, send params to the param server
      sleep(0.1); // delay (seconds)
      mavrosflight_->param.set_param_value("MAG_A11_COMP", mag_.a11());
      mavrosflight_->param.set_param_value("MAG_A12_COMP", mag_.a12());
      mavrosflight_->param.set_param_value("MAG_A13_COMP", mag_.a13());
      mavrosflight_->param.set_param_value("MAG_A21_COMP", mag_.a21());
      mavrosflight_->param.set_param_value("MAG_A22_COMP", mag_.a22());
      mavrosflight_->param.set_param_value("MAG_A23_COMP", mag_.a23());
      mavrosflight_->param.set_param_value("MAG_A31_COMP", mag_.a31());
      mavrosflight_->param.set_param_value("MAG_A32_COMP", mag_.a32());
      mavrosflight_->param.set_param_value("MAG_A33_COMP", mag_.a33());
      mavrosflight_->param.set_param_value("MAG_X_BIAS", mag_.bx());
      mavrosflight_->param.set_param_value("MAG_Y_BIAS", mag_.by());
      mavrosflight_->param.set_param_value("MAG_Z_BIAS", mag_.bz());

      ROS_WARN("Write params to save new magnetometer calibration!");
    }
  }

  bool valid = mag_.correct(mag, &mag_msg.magnetic_field.x, &mag_msg.magnetic_field.y, &mag_msg.magnetic_field.z);

  if(mag_calibrate_srv_.getService().empty())
  {
    mag_calibrate_srv_ = nh_.advertiseService("calibrate_mag", &fcuIO::calibrateMagSrvCallback, this);
  }

  if (valid)
  {
    if (mag_pub_.getTopic().empty())
    {
      mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("magnetometer", 1);
    }
    mag_pub_.publish(mag_msg);
  }
}

void fcuIO::handle_small_sonar(const mavlink_message_t &msg)
{
  mavlink_small_sonar_t sonar;
  mavlink_msg_small_sonar_decode(&msg, &sonar);

  sensor_msgs::Range alt_msg;
  alt_msg.header.stamp = ros::Time::now();

  // MB1242 returns in cm
  alt_msg.max_range = sonar.max_range;
  alt_msg.min_range = sonar.min_range;
  alt_msg.range = sonar.range;

  alt_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  alt_msg.field_of_view = 1.0472;  // approx 60 deg

  if (sonar_pub_.getTopic().empty())
  {
    sonar_pub_ = nh_.advertise<sensor_msgs::Range>("sonar/data", 1);
  }
  sonar_pub_.publish(alt_msg);
}

void fcuIO::commandCallback(fcu_common::Command::ConstPtr msg)
{
  //! \todo these are hard-coded to match right now; may want to replace with something more robust
  OFFBOARD_CONTROL_MODE mode = (OFFBOARD_CONTROL_MODE)msg->mode;
  OFFBOARD_CONTROL_IGNORE ignore = (OFFBOARD_CONTROL_IGNORE)msg->ignore;

  float x = msg->x;
  float y = msg->y;
  float z = msg->z;
  float F = msg->F;

  switch (mode)
  {
    case MODE_PASS_THROUGH:
      x = saturate(x, -1.0f, 1.0f);
      y = saturate(y, -1.0f, 1.0f);
      z = saturate(z, -1.0f, 1.0f);
      F = saturate(F, 0.0f, 1.0f);
      break;
    case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
    case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
      F = saturate(F, 0.0f, 1.0f);
      break;
    case MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
      break;
  }

  mavlink_message_t mavlink_msg;
  mavlink_msg_offboard_control_pack(1, 50, &mavlink_msg, mode, ignore, x, y, z, F);
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

bool fcuIO::paramSaveToFileCallback(ParamFile::Request &req, ParamFile::Response &res)
{
  res.success = mavrosflight_->param.save_to_file(req.filename);
  return true;
}

bool fcuIO::paramLoadFromFileCallback(ParamFile::Request &req, ParamFile::Response &res)
{
  res.success = mavrosflight_->param.load_from_file(req.filename);
  return true;
}

bool fcuIO::calibrateImuBiasSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_command_int_pack(1, 50, &msg, 1, MAV_COMP_ID_ALL, 0, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 1.0, 0.0, 0.0,
                               0.0, 1, 0, 0.0);
  mavrosflight_->serial.send_message(msg);

  res.success = true;
  return true;
}

bool fcuIO::calibrateRCTrimSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_command_int_pack(1, 50, &msg, 1, MAV_COMP_ID_ALL, 0, MAV_CMD_DO_RC_CALIBRATION, 0, 0, 0, 0, 0, 0, 0, 0,
                               0);
  mavrosflight_->serial.send_message(msg);
  res.success = true;
  return true;
}

void fcuIO::paramTimerCallback(const ros::TimerEvent &e)
{
  if (mavrosflight_->param.got_all_params())
  {
    param_timer_.stop();
    ROS_INFO("Received all parameters");
  }
  else
  {
    mavrosflight_->param.request_params();
    ROS_INFO("Received %d of %d parameters. Requesting missing parameters...",
             mavrosflight_->param.get_params_received(), mavrosflight_->param.get_num_params());
  }
}

bool fcuIO::calibrateImuTempSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  // First, reset the previous calibration
  mavrosflight_->param.set_param_value("ACC_X_TEMP_COMP", 0);
  mavrosflight_->param.set_param_value("ACC_Y_TEMP_COMP", 0);
  mavrosflight_->param.set_param_value("ACC_Z_TEMP_COMP", 0);
  mavrosflight_->param.set_param_value("ACC_X_BIAS", 0);
  mavrosflight_->param.set_param_value("ACC_Y_BIAS", 0);
  mavrosflight_->param.set_param_value("ACC_Z_BIAS", 0);

  // tell the IMU to start a temperature calibration
  imu_.start_temp_calibration();
  ROS_WARN("IMU temperature calibration started");

  res.success = true;
  return true;
}


bool fcuIO::calibrateMagSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  // reset the previous calibration
  sleep(0.1); // delay (seconds)
  mavrosflight_->param.set_param_value("MAG_A11_COMP", 1);
  mavrosflight_->param.set_param_value("MAG_A12_COMP", 0);
  mavrosflight_->param.set_param_value("MAG_A13_COMP", 0);
  mavrosflight_->param.set_param_value("MAG_A21_COMP", 0);
  mavrosflight_->param.set_param_value("MAG_A22_COMP", 1);
  mavrosflight_->param.set_param_value("MAG_A23_COMP", 0);
  mavrosflight_->param.set_param_value("MAG_A31_COMP", 0);
  mavrosflight_->param.set_param_value("MAG_A32_COMP", 0);
  mavrosflight_->param.set_param_value("MAG_A33_COMP", 1);
  mavrosflight_->param.set_param_value("MAG_X_BIAS", 0);
  mavrosflight_->param.set_param_value("MAG_Y_BIAS", 0);
  mavrosflight_->param.set_param_value("MAG_Z_BIAS", 0);

  // tell the magnetometer to start a temperature calibration
  mag_.start_calibration();
}

bool fcuIO::calibrateAirspeedSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_command_int_pack(1, 50, &msg, 1, MAV_COMP_ID_ALL,
                               0, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0, 0, 0);
  mavrosflight_->serial.send_message(msg);
  res.success = true;
  return true;
}

bool fcuIO::calibrateBaroSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_command_int_pack(1, 50, &msg, 1, MAV_COMP_ID_ALL,
                               0, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 1, 0, 0, 0, 0, 0);
  mavrosflight_->serial.send_message(msg);
  res.success = true;
  return true;
}


bool fcuIO::calibrateAirspeedSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_command_int_pack(1, 50, &msg, 1, MAV_COMP_ID_ALL,
                               0, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0, 0, 0);
  mavrosflight_->serial.send_message(msg);
  res.success = true;
  return true;
}

bool fcuIO::calibrateBaroSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_command_int_pack(1, 50, &msg, 1, MAV_COMP_ID_ALL,
                               0, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 1, 0, 0, 0, 0, 0);
  mavrosflight_->serial.send_message(msg);
  res.success = true;
  return true;
}

bool fcuIO::rebootSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_command_int_pack(1, 50, &msg, 1, MAV_COMP_ID_ALL,
                               0, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  mavrosflight_->serial.send_message(msg);
  res.success = true;
  return true;
}

} // namespace fcu_io

