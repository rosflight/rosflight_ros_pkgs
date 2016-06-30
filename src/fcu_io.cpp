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
  servo_output_raw_pub_ = nh.advertise<fcu_common::ServoOutputRaw>("servo_output_raw", 1);
  rc_raw_pub_ = nh.advertise<fcu_common::ServoOutputRaw>("rc_raw", 1);
  diff_pressure_pub_ = nh.advertise<sensor_msgs::FluidPressure>("diff_pressure", 1);
  temperature_pub_ = nh.advertise<sensor_msgs::Temperature>("temperature", 1);
  baro_pub_ = nh.advertise<std_msgs::Float32>("baro/alt", 1);

  param_get_srv_ = nh.advertiseService("param_get", &fcuIO::paramGetSrvCallback, this);
  param_set_srv_ = nh.advertiseService("param_set", &fcuIO::paramSetSrvCallback, this);
  param_write_srv_ = nh.advertiseService("param_write", &fcuIO::paramWriteSrvCallback, this);

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

  start_time_ = ros::Time::now().toSec();
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
  }
}

void fcuIO::on_new_param_received(std::string name, double value)
{https://github.com/simondlevy/
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
  double now = ros::Time::now().toSec() - start_time_;
  static bool calibrated = false;

  // read temperature of accel chip for temperature compensation calibration
  double temperature = ((float)imu.temp/340.0 + 36.53);

  static double calibration_time = 5.0; // seconds to record data for temperature compensation
  static double deltaT = 1.0; // number of degrees required for a temperature calibration

  static double Tmin = 1000; // minimum temperature seen
  static double Tmax = -1000; // maximum temperature seen

  // one for each accel axis
  static std::deque<Eigen::Vector3d> A(0);
  static std::deque<double> B(0);
  static Eigen::Vector2d x[3];

  // if still in calibration mode
  if(now < calibration_time || (Tmax - Tmin) < deltaT)
  {
    ROS_INFO("IMU CALIBRATING, time %f, deltaT %f", now, Tmax-Tmin);
    Eigen::Vector3d measurement;
    measurement << imu.xacc, imu.yacc, imu.zacc - 4096; // need a better way to know the z-axis offset
    A.push_back(measurement);
    B.push_back(temperature);
    if(temperature < Tmin)
    {
      Tmin = temperature;
    }
    if(temperature > Tmax)
    {
      Tmax = temperature;
    }
  }
  else if(!calibrated)
  {
    ROS_INFO("IMU DONE CALIBRATING, current time = %f, start = %f", now, calibration_time);
    for(int i = 0; i < 3; i++)
    {
      Eigen::MatrixX2d Amat;
      Eigen::VectorXd Bmat;
      Amat.resize(A.size(),2);
      Bmat.resize(B.size());

      // put the data into and Eigen Matrix for linear algebra
      for(int j = 0; j<A.size(); j++)
      {
        Amat(j,0) = A[j](i);
        Amat(j,1) = 1.0;
        Bmat(j) = B[j];
      }

      // Perform Least-Squares on the data
      x[i] = Amat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(Bmat);
    }
    calibrated = true;
  }
  else
  {
    sensor_msgs::Imu out_msg;

    out_msg.header.stamp = ros::Time::now(); //! \todo time synchronization

    float accel_scale = 0.002349f;
    out_msg.linear_acceleration.x = (imu.xacc - (temperature)*x[0](0) - x[0](1)) * accel_scale;
    out_msg.linear_acceleration.y = (imu.yacc - (temperature)*x[1](0) - x[1](1)) * accel_scale;
    out_msg.linear_acceleration.z = (imu.zacc - (temperature)*x[2](0) - x[2](1)) * accel_scale;

    float gyro_scale = .004256f;
    out_msg.angular_velocity.x = imu.xgyro * gyro_scale;
    out_msg.angular_velocity.y = imu.ygyro * gyro_scale;
    out_msg.angular_velocity.z = imu.zgyro * gyro_scale;

    imu_pub_.publish(out_msg);
  }
}

void fcuIO::handle_servo_output_raw_msg(const mavlink_message_t &msg)
{
  mavlink_servo_output_raw_t servo;
  mavlink_msg_servo_output_raw_decode(&msg, &servo);

  fcu_common::ServoOutputRaw out_msg;
  out_msg.header.stamp = ros::Time::now(); //! \todo time synchronization
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
  out_msg.header.stamp = ros::Time::now();
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

  const double P_min = -1.0f;
  const double P_max = 1.0f;
  const double PSI_to_Pa = 6894.757f;

  static int calibration_counter = 0;
  static int calibration_count = 100;
  static double _diff_pres_offset = 0.0;

  // conversion from pixhawk source code
  double temp = ((200.0f * diff.temperature) / 2047) - 50;

  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = ros::Time::now();
  temp_msg.temperature = temp;
  temperature_pub_.publish(temp_msg);

  /*
   * this equation is an inversion of the equation in the
   * pressure transfer function figure on page 4 of the datasheet
   * We negate the result so that positive differential pressures
   * are generated when the bottom port is used as the static
   * port on the pitot and top port is used as the dynamic port
   */
  double diff_press_PSI = -((diff.diff_pressure - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min);
  double diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;
  if (calibration_counter > calibration_count)
  {
    diff_press_pa_raw -= _diff_pres_offset;

    sensor_msgs::FluidPressure pressure_msg;
    pressure_msg.header.stamp = ros::Time::now();
    pressure_msg.fluid_pressure = diff_press_pa_raw;
    diff_pressure_pub_.publish(pressure_msg);
  }
  else if (calibration_counter == calibration_count)
  {
    _diff_pres_offset = _diff_pres_offset/calibration_count;
    calibration_counter++;
  }
  else
  {
    _diff_pres_offset += diff_press_pa_raw;
    calibration_counter++;
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

  double pressure = baro.pressure;
  double temperature = baro.temperature;

  // calibration variables
  static int calibration_counter = 0;
  static double calibration_sum = 0;
  static int settling_count = 20; // settle for a second or so
  static int calibration_count = 20;

  // offsets and filters
  static double prev_alt = 0.0;
  static double alt_alpha_ = 0.3; // really slow
  static double alt_ground = 0;

  if( calibration_counter > calibration_count + settling_count)
  {
    double alt_tmp = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 4430.0f; // in meters

    // offset calculated ground altitude
    alt_tmp -= alt_ground;

    // LPF measurements
    double altitude = alt_alpha_*alt_tmp + (1.0 - alt_alpha_)*prev_alt;
    prev_alt = altitude;

    // publish measurement
    std_msgs::Float32 alt_msg;
    alt_msg.data = altitude;
    baro_pub_.publish(alt_msg);
  }
  if (calibration_counter < settling_count)
  {
    calibration_counter++;
  }
  else if (calibration_counter < settling_count + calibration_count)
  {
    double measurement = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 4430.0f;
    calibration_sum += measurement;
    calibration_counter++;
  }
  else if(calibration_counter == settling_count + calibration_count)
  {
    alt_ground = calibration_sum/calibration_count;
    ROS_INFO_STREAM("BARO CALIBRATED " << alt_ground << " meters above sea level");
    calibration_counter++;
  }
}

void fcuIO::commandCallback(fcu_common::ExtendedCommand::ConstPtr msg)
{
  //! \todo these are hard-coded to match right now; may want to replace with something more robust
  OFFBOARD_CONTROL_MODE mode = (OFFBOARD_CONTROL_MODE) msg->mode;
  OFFBOARD_CONTROL_IGNORE ignore = (OFFBOARD_CONTROL_IGNORE) msg->ignore;

  int v1 = (int) (msg->value1 * 1000);
  int v2 = (int) (msg->value2 * 1000);
  int v3 = (int) (msg->value3 * 1000);
  int v4 = (int) (msg->value4 * 1000);

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

} // namespace fcu_io
