/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
 * Copyright (c) 2024 Ian Reid, BYU MAGICC Lab.
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

#include "rosflight_sim/gz_compat.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <rclcpp/logging.hpp>
#include <rosflight_sim/sil_board.hpp>

#include <iostream>

namespace rosflight_sim
{
SILBoard::SILBoard()
    : UDPBoard()
    // , bias_generator_(std::chrono::system_clock::now().time_since_epoch().count()) // Uncomment if you would like to
                                                                                      // have different biases for the sensors
                                                                                      // on each flight. Delete next line.
    , bias_generator_(0)
    , noise_generator_(std::chrono::system_clock::now().time_since_epoch().count())
{}

void SILBoard::init_board() { boot_time_ = GZ_COMPAT_GET_SIM_TIME(world_); }

constexpr double rad2Deg(double x) { return 180.0 / M_PI * x; }
constexpr double deg2Rad(double x) { return M_PI / 180.0 * x; }

void SILBoard::gazebo_setup(gazebo::physics::LinkPtr link, gazebo::physics::WorldPtr world,
                            gazebo::physics::ModelPtr model, rclcpp::Node::SharedPtr node,
                            std::string mav_type)
{
  link_ = link;
  world_ = world;
  model_ = model;
  node_ = node;
  mav_type_ = std::move(mav_type);

  auto bind_host = node_->get_parameter_or<std::string>("gazebo_host", "localhost");
  int bind_port = node_->get_parameter_or<int>("gazebo_port", 14525);
  auto remote_host = node_->get_parameter_or<std::string>("ROS_host", "localhost");
  int remote_port = node_->get_parameter_or<int>("ROS_port", 14520);

  set_ports(bind_host, bind_port, remote_host, remote_port);
  gzmsg << "ROSflight SIL Conneced to " << remote_host << ":" << remote_port << " from "
        << bind_host << ":" << bind_port << "\n";

  // TODO: These params need to be updated with empirically derived values, using the latest
  //   hardware (i.e. not the cheap boards with the cheap sensors)

  // Get communication delay parameters, in nanoseconds
  serial_delay_ns_ = node_->get_parameter_or<long>("serial_delay_ns", 0.006 * 1e9);

  // Get Sensor Parameters
  gyro_stdev_ = node_->get_parameter_or<double>("gyro_stdev", 0.00226);
  gyro_bias_range_ = node_->get_parameter_or<double>("gyro_bias_range", 0.25);
  gyro_bias_walk_stdev_ = node_->get_parameter_or<double>("gyro_bias_walk_stdev", 0.033);
  k_gyro_ = node_->get_parameter_or<double>("k_gyro", 20.0);
  gyro_bias_model_tau_ = node_->get_parameter_or<double>("gyro_bias_model_tau", 400.0);
  double bias_model_x0 = node_->get_parameter_or<double>("bias_model_x0", 0.003);
  double bias_model_y0 = node_->get_parameter_or<double>("bias_model_y0", -0.002);
  double bias_model_z0 = node_->get_parameter_or<double>("bias_model_z0", 0.001);

  GZ_COMPAT_SET_X(bias_instability_, bias_model_x0);
  GZ_COMPAT_SET_Y(bias_instability_, bias_model_y0);
  GZ_COMPAT_SET_Z(bias_instability_, bias_model_z0);
  
  GZ_COMPAT_SET_X(gyro_bias_eta_, 0.0);
  GZ_COMPAT_SET_Y(gyro_bias_eta_, 0.0);
  GZ_COMPAT_SET_Z(gyro_bias_eta_, 0.0);

  acc_stdev_ = node_->get_parameter_or<double>("acc_stdev", 0.2);
  acc_bias_range_ = node_->get_parameter_or<double>("acc_bias_range", 0.6);
  acc_bias_walk_stdev_ = node_->get_parameter_or<double>("acc_bias_walk_stdev", 0.00001);

  mag_stdev_ = node_->get_parameter_or<double>("mag_stdev", 3000/1e9); // from nano tesla to tesla
  k_mag_ = node_->get_parameter_or<double>("k_mag", 7.0);

  baro_stdev_ = node_->get_parameter_or<double>("baro_stdev", 4.0);
  baro_bias_range_ = node_->get_parameter_or<double>("baro_bias_range", 500);
  baro_bias_walk_stdev_ = node_->get_parameter_or<double>("baro_bias_walk_stdev", 0.1);

  airspeed_stdev_ = node_->get_parameter_or<double>("airspeed_stdev", 1.15);
  airspeed_bias_range_ = node_->get_parameter_or<double>("airspeed_bias_range", 0.15);
  airspeed_bias_walk_stdev_ = node_->get_parameter_or<double>("airspeed_bias_walk_stdev", 0.001);

  sonar_stdev_ = node_->get_parameter_or<double>("sonar_stdev", 0.03);
  sonar_min_range_ = node_->get_parameter_or<double>("sonar_min_range", 0.25);
  sonar_max_range_ = node_->get_parameter_or<double>("sonar_max_range", 8.0);

  imu_update_rate_ = node_->get_parameter_or<double>("imu_update_rate", 1000.0);
  imu_update_period_us_ = (uint64_t) (1e6 / imu_update_rate_);

  mag_update_rate_ = node_->get_parameter_or<double>("mag_update_rate", 50.0);
  mag_update_period_us_ = (uint64_t) (1e6 / mag_update_rate_);

  GZ_COMPAT_SET_X(mag_gauss_markov_eta_, 0.0);
  GZ_COMPAT_SET_Y(mag_gauss_markov_eta_, 0.0);
  GZ_COMPAT_SET_Z(mag_gauss_markov_eta_, 0.0);

  gnss_update_rate_ = node_->get_parameter_or<double>("gnss_update_rate", 10.0);
  gnss_update_period_us_ = (uint64_t) (1e6 / gnss_update_rate_);

  baro_update_rate_ = node_->get_parameter_or<double>("baro_update_rate", 50.0);
  baro_update_period_us_ = (uint64_t) (1e6 / baro_update_rate_);

  diff_pressure_update_rate_ = node_->get_parameter_or<double>("diff_pressure_update_rate", 50.0);
  diff_pressure_update_period_us_ = (uint64_t) (1e6 / diff_pressure_update_rate_);

  sonar_update_rate_ = node_->get_parameter_or<double>("sonar_update_rate", 50.0);
  sonar_update_period_us_ = (uint64_t) (1e6 / sonar_update_rate_);

  rc_update_rate_ = node_->get_parameter_or<double>("rc_update_rate", 50.0);
  rc_update_period_us_ = (uint64_t) (1e6 / rc_update_rate_);

  battery_update_rate_ = node_->get_parameter_or<double>("battery_update_rate", 5.0);
  battery_update_period_us_ = (uint64_t) (1e6 / battery_update_rate_);

  mass_ = node_->get_parameter_or<double>("mass", 2.28);
  rho_ = node_->get_parameter_or<double>("rho", 1.225);

  // Get the desired altitude at the ground (for baro and LLA)

  origin_altitude_ = node_->get_parameter_or<double>("origin_altitude", 1387.0);
  origin_latitude_ = node_->get_parameter_or<double>("origin_latitude", 40.2463724);
  origin_longitude_ = node_->get_parameter_or<double>("origin_longitude", -111.6474138);

  horizontal_gnss_stdev_ = node_->get_parameter_or<double>("horizontal_gnss_stdev", 0.21);
  vertical_gnss_stdev_ = node_->get_parameter_or<double>("vertical_gnss_stdev", 0.4);
  gnss_velocity_stdev_ = node_->get_parameter_or<double>("gnss_velocity_stdev", 0.01);
  k_gnss_ = node_->get_parameter_or<double>("k_gnss", 1.0/1100);
  
  GZ_COMPAT_SET_X(gnss_gauss_markov_eta_, 0.0);
  GZ_COMPAT_SET_Y(gnss_gauss_markov_eta_, 0.0);
  GZ_COMPAT_SET_Z(gnss_gauss_markov_eta_, 0.0);

  // Configure Noise
  normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  uniform_distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);

  gravity_ = GZ_COMPAT_GET_GRAVITY(world_);

  // Initialize the Sensor Biases
  GZ_COMPAT_SET_X(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Y(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Z(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_X(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Y(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Z(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));
  baro_bias_ = baro_bias_range_ * uniform_distribution_(bias_generator_);
  airspeed_bias_ = airspeed_bias_range_ * uniform_distribution_(bias_generator_);

  prev_vel_1_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  prev_vel_2_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  prev_vel_3_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  last_time_ = GZ_COMPAT_GET_SIM_TIME(world_);
  next_imu_update_time_us_ = 0;
  next_mag_update_time_us_ = 0;
}

// clock

uint32_t SILBoard::clock_millis()
{
  uint32_t millis = (uint32_t) ((GZ_COMPAT_GET_SIM_TIME(world_) - boot_time_).Double() * 1e3);
  return millis;
}

uint64_t SILBoard::clock_micros()
{
  uint64_t micros = (uint64_t) ((GZ_COMPAT_GET_SIM_TIME(world_) - boot_time_).Double() * 1e6);
  return micros;
}

uint8_t SILBoard::serial_read()
{
  auto next_message = serial_delay_queue_.front();
  serial_delay_queue_.pop();
  return std::get<1>(next_message);
}

uint16_t SILBoard::serial_bytes_available()
{
  // Get current time. Doesn't use ROS time as ROS time proved to be inconsistent and lead to slower
  // serial communication.
  auto current_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();

  // Get available serial_read messages from the firmware
  if (UDPBoard::serial_bytes_available()) {
    serial_delay_queue_.emplace(current_time, UDPBoard::serial_read());
  }

  // Determine if there are any serial_read messages that are ready for processing
  return !serial_delay_queue_.empty()
    && (current_time - std::get<0>(serial_delay_queue_.front())) > serial_delay_ns_;
}

// sensors
/// TODO these sensors have noise, no bias
/// noise params are hard coded
void SILBoard::sensors_init()
{
  // Initialize the Biases
  GZ_COMPAT_SET_X(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Y(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Z(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_X(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Y(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Z(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));

  // Gazebo coordinates is NWU and Earth's magnetic field is defined in NED, hence the negative signs
  auto inclination_ = node_->get_parameter_or<double>("inclination", 1.139436457);
  auto declination_ = node_->get_parameter_or<double>("declination", 0.1857972802);
  double total_intensity = node_->get_parameter_or<double>("total_intensity", 50716.3 / 1e9); // nanoTesla converted to tesla.
  
  GZ_COMPAT_SET_Z(inertial_magnetic_field_, sin(-inclination_));
  GZ_COMPAT_SET_X(inertial_magnetic_field_, cos(-inclination_) * cos(-declination_));
  GZ_COMPAT_SET_Y(inertial_magnetic_field_, cos(-inclination_) * sin(-declination_));
  inertial_magnetic_field_ = inertial_magnetic_field_.Normalized();
  inertial_magnetic_field_ *= total_intensity;

  using SC = gazebo::common::SphericalCoordinates;
  using Ang = ignition::math::Angle;
  sph_coord_.SetSurfaceType(SC::SurfaceType::EARTH_WGS84);
  sph_coord_.SetLatitudeReference(Ang(deg2Rad(origin_latitude_)));
  sph_coord_.SetLongitudeReference(Ang(deg2Rad(origin_longitude_)));
  sph_coord_.SetElevationReference(origin_altitude_);
  // Force x-axis to be north-aligned. I promise, I will change everything to ENU in the next commit
  sph_coord_.SetHeadingOffset(Ang(M_PI / 2.0));
}

bool SILBoard::imu_has_new_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_imu_update_time_us_) {
    next_imu_update_time_us_ = now_us + imu_update_period_us_;
    return true;
  } else {
    return false;
  }
}

bool SILBoard::mag_has_new_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_mag_update_time_us_) {
    next_mag_update_time_us_ = now_us + mag_update_period_us_;
    return true;
  } else {
    return false;
  }
}

bool SILBoard::gnss_has_new_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_gnss_update_time_us_) {
    next_gnss_update_time_us_ = now_us + gnss_update_period_us_;
    return true;
  } else {
    return false;
  }
}

bool SILBoard::baro_has_new_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_baro_update_time_us_) {
    next_baro_update_time_us_ = now_us + baro_update_period_us_;
    return true;
  } else {
    return false;
  }
}

bool SILBoard::diff_pressure_has_new_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_diff_pressure_update_time_us_) {
    next_diff_pressure_update_time_us_ = now_us + diff_pressure_update_period_us_;
    return true;
  } else {
    return false;
  }
}

bool SILBoard::sonar_has_new_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_sonar_update_time_us_) {
    next_sonar_update_time_us_ = now_us + sonar_update_period_us_;
    return true;
  } else {
    return false;
  }
}

bool SILBoard::rc_has_new_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_rc_update_time_us_) {
    next_rc_update_time_us_ = now_us + rc_update_period_us_;
    return true;
  } else {
    return false;
  }
}

bool SILBoard::battery_has_new_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_battery_update_time_us_) {
    next_battery_update_time_us_ = now_us + battery_update_period_us_;
    return true;
  } else {
    return false;
  }
}

bool SILBoard::imu_read(float accel[3], float * temperature, float gyro[3], uint64_t * time_us)
{
  GazeboQuaternion q_I_NWU = GZ_COMPAT_GET_ROT(GZ_COMPAT_GET_WORLD_POSE(link_));
  GazeboVector current_vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  GazeboVector y_acc;
  GazeboPose local_pose = GZ_COMPAT_GET_WORLD_POSE(link_);

  // this is James's egregious hack to overcome wild imu while sitting on the ground
  if (GZ_COMPAT_GET_LENGTH(current_vel) < 0.05) {
    y_acc = q_I_NWU.RotateVectorReverse(-gravity_);
  } else if (local_pose.Z() < 0.3) {
    y_acc = q_I_NWU.RotateVectorReverse(GZ_COMPAT_GET_WORLD_LINEAR_ACCEL(link_) - gravity_);
  } else {
    y_acc.Set(f_x / mass_, -f_y / mass_, -f_z / mass_);
  }

  // Apply normal noise (only if armed, because most of the noise comes from motors
  if (motors_spinning()) {
    GZ_COMPAT_SET_X(y_acc,
                    GZ_COMPAT_GET_X(y_acc) + acc_stdev_ * normal_distribution_(noise_generator_));
    GZ_COMPAT_SET_Y(y_acc,
                    GZ_COMPAT_GET_Y(y_acc) + acc_stdev_ * normal_distribution_(noise_generator_));
    GZ_COMPAT_SET_Z(y_acc,
                    GZ_COMPAT_GET_Z(y_acc) + acc_stdev_ * normal_distribution_(noise_generator_));
  }

  // Perform bias Walk for biases
  GZ_COMPAT_SET_X(acc_bias_,
                  GZ_COMPAT_GET_X(acc_bias_)
                    + acc_bias_walk_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Y(acc_bias_,
                  GZ_COMPAT_GET_Y(acc_bias_)
                    + acc_bias_walk_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Z(acc_bias_,
                  GZ_COMPAT_GET_Z(acc_bias_)
                    + acc_bias_walk_stdev_ * normal_distribution_(noise_generator_));

  // Add constant Bias to measurement
  GZ_COMPAT_SET_X(y_acc, GZ_COMPAT_GET_X(y_acc) + GZ_COMPAT_GET_X(acc_bias_));
  GZ_COMPAT_SET_Y(y_acc, GZ_COMPAT_GET_Y(y_acc) + GZ_COMPAT_GET_Y(acc_bias_));
  GZ_COMPAT_SET_Z(y_acc, GZ_COMPAT_GET_Z(y_acc) + GZ_COMPAT_GET_Z(acc_bias_));

  // Convert to NED for output
  accel[0] = GZ_COMPAT_GET_X(y_acc);
  accel[1] = (float) -GZ_COMPAT_GET_Y(y_acc);
  accel[2] = (float) -GZ_COMPAT_GET_Z(y_acc);

  GazeboVector y_gyro = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);

  if (GZ_COMPAT_GET_LENGTH(current_vel) < 0.05) {
    GZ_COMPAT_SET_X(y_gyro, 0.0);
    GZ_COMPAT_SET_Y(y_gyro, 0.0);
    GZ_COMPAT_SET_Z(y_gyro, 0.0);
  }

  // Normal Noise from motors
  if (motors_spinning()) {
    GZ_COMPAT_SET_X(y_gyro,
                    GZ_COMPAT_GET_X(y_gyro) + gyro_stdev_ * normal_distribution_(noise_generator_));
    GZ_COMPAT_SET_Y(y_gyro,
                    GZ_COMPAT_GET_Y(y_gyro) + gyro_stdev_ * normal_distribution_(noise_generator_));
    GZ_COMPAT_SET_Z(y_gyro,
                    GZ_COMPAT_GET_Z(y_gyro) + gyro_stdev_ * normal_distribution_(noise_generator_));
  }

  // bias Walk for bias
  float T_s = 1.0/imu_update_rate_;
  GazeboVector noise;
  GZ_COMPAT_SET_X(noise, gyro_bias_walk_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Y(noise, gyro_bias_walk_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Z(noise, gyro_bias_walk_stdev_ * normal_distribution_(noise_generator_));
  gyro_bias_eta_ = std::exp(-k_gyro_*T_s) * gyro_bias_eta_ + T_s*noise;

  // Calculate bias instability model.
  bias_instability_ = bias_model();


  // Apply Constant Bias
  GZ_COMPAT_SET_X(y_gyro, GZ_COMPAT_GET_X(y_gyro) + GZ_COMPAT_GET_X(gyro_bias_));
  GZ_COMPAT_SET_Y(y_gyro, GZ_COMPAT_GET_Y(y_gyro) + GZ_COMPAT_GET_Y(gyro_bias_));
  GZ_COMPAT_SET_Z(y_gyro, GZ_COMPAT_GET_Z(y_gyro) + GZ_COMPAT_GET_Z(gyro_bias_));

  // Apply modeled bias instability
  GZ_COMPAT_SET_X(y_gyro, GZ_COMPAT_GET_X(y_gyro) + GZ_COMPAT_GET_X(bias_instability_));
  GZ_COMPAT_SET_Y(y_gyro, GZ_COMPAT_GET_Y(y_gyro) + GZ_COMPAT_GET_Y(bias_instability_));
  GZ_COMPAT_SET_Z(y_gyro, GZ_COMPAT_GET_Z(y_gyro) + GZ_COMPAT_GET_Z(bias_instability_));
  
  // Apply Bias walk
  GZ_COMPAT_SET_X(y_gyro, GZ_COMPAT_GET_X(y_gyro) + GZ_COMPAT_GET_X(gyro_bias_eta_));
  GZ_COMPAT_SET_Y(y_gyro, GZ_COMPAT_GET_Y(y_gyro) + GZ_COMPAT_GET_Y(gyro_bias_eta_));
  GZ_COMPAT_SET_Z(y_gyro, GZ_COMPAT_GET_Z(y_gyro) + GZ_COMPAT_GET_Z(gyro_bias_eta_));

  // Convert to NED for output
  gyro[0] = GZ_COMPAT_GET_X(y_gyro);
  gyro[1] = (float) -GZ_COMPAT_GET_Y(y_gyro);
  gyro[2] = (float) -GZ_COMPAT_GET_Z(y_gyro);

  (*temperature) = 27.0 + 273.15;
  (*time_us) = clock_micros();
  return true;
}

ignition::math::Vector3d SILBoard::bias_model() 
{
  double T = 1/imu_update_rate_;

  double alpha = T/(T+gyro_bias_model_tau_);
  
  GZ_COMPAT_SET_X(bias_instability_, GZ_COMPAT_GET_X(bias_instability_) * (1-alpha));
  GZ_COMPAT_SET_Y(bias_instability_, GZ_COMPAT_GET_Y(bias_instability_) * (1-alpha));
  GZ_COMPAT_SET_Z(bias_instability_, GZ_COMPAT_GET_Z(bias_instability_) * (1-alpha));

   return bias_instability_;
}

void SILBoard::imu_not_responding_error()
{
  RCLCPP_ERROR(node_->get_logger(), "[gazebo_rosflight_sil] imu not responding");
}

bool SILBoard::mag_read(float mag[3])
{
  float T_s = 1.0/mag_update_rate_;
  
  GazeboPose I_to_B = GZ_COMPAT_GET_WORLD_POSE(link_);

  GazeboVector y_mag =
    GZ_COMPAT_GET_ROT(I_to_B).RotateVector(inertial_magnetic_field_) + mag_gauss_markov_eta_;
  
  GazeboVector noise;
  GZ_COMPAT_SET_X(noise, mag_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Y(noise, mag_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Z(noise, mag_stdev_ * normal_distribution_(noise_generator_));
  mag_gauss_markov_eta_ = std::exp(-k_mag_*T_s) * mag_gauss_markov_eta_ + T_s*noise;

  // Convert measurement to NED
  mag[0] = (float) GZ_COMPAT_GET_X(y_mag);
  mag[1] = (float) -GZ_COMPAT_GET_Y(y_mag);
  mag[2] = (float) -GZ_COMPAT_GET_Z(y_mag);

  return true;
}

bool SILBoard::mag_present() { return true; }

bool SILBoard::baro_present() { return true; }

bool SILBoard::baro_read(float * pressure, float * temperature)
{
  // pull z measurement out of Gazebo
  GazeboPose current_state_NWU = GZ_COMPAT_GET_WORLD_POSE(link_);

  // Invert measurement model for pressure and temperature
  double alt = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(current_state_NWU)) + origin_altitude_;

  // Convert to the true pressure reading
  double y_baro = 101325.0f
    * (float) pow((1 - 2.25694e-5 * alt), 5.2553); // Add these parameters to the parameters.

  // Add noise
  y_baro += baro_stdev_ * normal_distribution_(noise_generator_);

  // Perform bias walk
  baro_bias_ += baro_bias_walk_stdev_ * normal_distribution_(noise_generator_);

  // Add bias walk
  y_baro += baro_bias_;

  (*pressure) = (float) y_baro;
  (*temperature) = 27.0f + 273.15f;

  return true;
}

bool SILBoard::diff_pressure_present()
{
  if (mav_type_ == "fixedwing") {
    return true;
  } else {
    return false;
  }
}

bool SILBoard::diff_pressure_read(float * diff_pressure, float * temperature)
{
  // Calculate Airspeed
  GazeboVector vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);

  double Va = GZ_COMPAT_GET_LENGTH(vel);

  // Invert Airpseed to get sensor measurement
  double y_as = rho_ * Va * Va / 2.0; // Page 130 in the UAV Book

  // Add noise
  y_as += airspeed_stdev_ * normal_distribution_(noise_generator_);
  airspeed_bias_ += airspeed_bias_walk_stdev_ * normal_distribution_(noise_generator_);
  y_as += airspeed_bias_;

  *diff_pressure = (float) y_as;
  *temperature = 27.0 + 273.15;

  return true;
}

bool SILBoard::sonar_present() { return true; }

bool SILBoard::sonar_read(float * range)
{
  GazeboPose current_state_NWU = GZ_COMPAT_GET_WORLD_POSE(link_);
  double alt = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(current_state_NWU));

  if (alt < sonar_min_range_) {
    *range = (float) sonar_min_range_;
  } else if (alt > sonar_max_range_) {
    *range = (float) sonar_max_range_;
  } else {
    *range = (float) (alt + sonar_stdev_ * normal_distribution_(noise_generator_));
  }

  return true;
}

bool SILBoard::battery_present() { return true; }

bool SILBoard::battery_read(float * voltage, float * current)
{
  *voltage = 15 * battery_voltage_multiplier;
  *current = 1 * battery_current_multiplier;
  return true;
}

void SILBoard::battery_voltage_set_multiplier(double multiplier)
{
  battery_voltage_multiplier = (float) multiplier;
}

void SILBoard::battery_current_set_multiplier(double multiplier)
{
  battery_current_multiplier = (float) multiplier;
}

// PWM
void SILBoard::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  rc_received_ = false;
  latestRC_.values[0] = 1500; // x
  latestRC_.values[1] = 1500; // y
  latestRC_.values[3] = 1500; // z
  latestRC_.values[2] = 1000; // F
  latestRC_.values[4] = 1000; // attitude override
  latestRC_.values[5] = 1000; // arm

  for (int & pwm_output : pwm_outputs_) {
    pwm_output = 1000;
  }

  rc_sub_ = node_->create_subscription<rosflight_msgs::msg::RCRaw>(
    "RC", 1, std::bind(&SILBoard::RC_callback, this, std::placeholders::_1));
  forces_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/forces_and_moments", 1, std::bind(&SILBoard::forces_callback, this, std::placeholders::_1));
}

void SILBoard::pwm_init_multi(const float *rate, uint32_t channels)
{
  // Only call it once, since we don't set the rate for each channel differently in the SIM board.
  // This works since the pwm_init doesn't use the arguments passed to it (in the SIL board)
  pwm_init(0, 0);
}

void SILBoard::forces_callback(const geometry_msgs::msg::TwistStamped & msg)
{

  f_x = msg.twist.linear.x;
  f_y = msg.twist.linear.y;
  f_z = msg.twist.linear.z;
}

float SILBoard::rc_read(uint8_t channel)
{
  if (rc_sub_->get_publisher_count() > 0) {
    return static_cast<float>(latestRC_.values[channel] - 1000) / 1000.0f;
  }

  // no publishers, set throttle low and center everything else
  if (channel == 2) {
    return 0.0;
  }

  return 0.5;
}

void SILBoard::pwm_write(uint8_t channel, float value)
{
  pwm_outputs_[channel] = 1000 + (uint16_t) (1000 * value);
}

void SILBoard::pwm_write_multi(float *value, uint32_t channels)
{
  for (int i=0; i<(int) channels; ++i) {
    pwm_write(i, value[i]);
  }
}

void SILBoard::pwm_disable()
{
  for (int i = 0; i < 14; i++) {
    pwm_write(i, 0);
  }
}

bool SILBoard::rc_lost() { return !rc_received_; }

// non-volatile memory
bool SILBoard::memory_read(void * dest, size_t len)
{
  std::string directory = "rosflight_memory" + std::string(node_->get_namespace());
  std::ifstream memory_file;
  memory_file.open(directory + "/mem.bin", std::ios::binary);

  if (!memory_file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to load rosflight memory file %s/mem.bin",
                 directory.c_str());
    return false;
  }

  memory_file.read((char *) dest, (long) len);
  memory_file.close();
  return true;
}

bool SILBoard::memory_write(const void * src, size_t len)
{
  std::string directory = "rosflight_memory" + std::string(node_->get_namespace());
  std::string mkdir_command = "mkdir -p " + directory;
  const int dir_err = system(mkdir_command.c_str());

  if (dir_err == -1) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to write rosflight memory file %s/mem.bin",
                 directory.c_str());
    return false;
  }

  std::ofstream memory_file;
  memory_file.open(directory + "/mem.bin", std::ios::binary);
  memory_file.write((char *) src, (long) len);
  memory_file.close();
  return true;
}

bool SILBoard::motors_spinning()
{
  if (pwm_outputs_[2] > 1100) {
    return true;
  } else {
    return false;
  }
}

bool SILBoard::backup_memory_read(void * dest, size_t len)
{
  if (len <= BACKUP_SRAM_SIZE) {
    memcpy(dest, backup_memory_, len);
    return true;
  } else {
    return false;
  }
}

void SILBoard::backup_memory_write(const void * src, size_t len)
{
  if (len < BACKUP_SRAM_SIZE) {
    memcpy(backup_memory_, src, len);
  }
}

void SILBoard::backup_memory_clear(size_t len)
{
  if (len < BACKUP_SRAM_SIZE) {
    memset(backup_memory_, 0, len);
  }
}

void SILBoard::RC_callback(const rosflight_msgs::msg::RCRaw & msg)
{
  rc_received_ = true;
  last_rc_message_ = node_->get_clock()->now();
  latestRC_ = msg;
}

bool SILBoard::gnss_present() { return true; }

bool SILBoard::gnss_read(rosflight_firmware::GNSSData * gnss,
                         rosflight_firmware::GNSSFull * gnss_full)
{
  using Vec3 = ignition::math::Vector3d;
  using Coord = gazebo::common::SphericalCoordinates::CoordinateType;

  double T_s = 1.0/gnss_update_rate_;

  GazeboPose local_pose = GZ_COMPAT_GET_WORLD_POSE(link_);
  Vec3 local_pos = GZ_COMPAT_GET_POS(local_pose) + gnss_gauss_markov_eta_;

  Vec3 pos_noise(horizontal_gnss_stdev_ * normal_distribution_(noise_generator_),
                 horizontal_gnss_stdev_ * normal_distribution_(noise_generator_),
                 vertical_gnss_stdev_ * normal_distribution_(noise_generator_));
  gnss_gauss_markov_eta_ = std::exp(-k_gnss_*T_s) * gnss_gauss_markov_eta_ + T_s*pos_noise;


  Vec3 local_vel = GZ_COMPAT_GET_WORLD_LINEAR_VEL(link_);
  Vec3 vel_noise(gnss_velocity_stdev_ * normal_distribution_(noise_generator_),
                 gnss_velocity_stdev_ * normal_distribution_(noise_generator_),
                 gnss_velocity_stdev_ * normal_distribution_(noise_generator_));
  local_vel += vel_noise;

  Vec3 ecef_pos = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::ECEF);
  Vec3 ecef_vel = sph_coord_.VelocityTransform(local_vel, Coord::LOCAL, Coord::ECEF);
  Vec3 lla = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::SPHERICAL);
  
  gnss->lat = (int) std::round(rad2Deg(lla.X()) * 1e7);
  gnss->lon = (int) std::round(rad2Deg(lla.Y()) * 1e7);
  gnss->height = (int) std::round(lla.Z() * 1e3);

  // For now, we have defined the Gazebo Local Frame as NWU.  This should be fixed in a future
  // commit
  gnss->vel_n = (int) std::round(local_vel.X() * 1e3);
  gnss->vel_e = (int) std::round(-local_vel.Y() * 1e3);
  gnss->vel_d = (int) std::round(-local_vel.Z() * 1e3);

  gnss->fix_type = rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_3D_FIX;
  gnss->time_of_week = GZ_COMPAT_GET_SIM_TIME(world_).Double() * 1000;
  gnss->time = GZ_COMPAT_GET_SIM_TIME(world_).Double();
  gnss->nanos =
    (uint64_t) std::round((GZ_COMPAT_GET_SIM_TIME(world_).Double() - (double) gnss->time) * 1e9);

  gnss->h_acc = (int) std::round(horizontal_gnss_stdev_ * 1000.0);
  gnss->v_acc = (int) std::round(vertical_gnss_stdev_ * 1000.0);

  gnss->ecef.x = (int) std::round(ecef_pos.X() * 100);
  gnss->ecef.y = (int) std::round(ecef_pos.Y() * 100);
  gnss->ecef.z = (int) std::round(ecef_pos.Z() * 100);
  gnss->ecef.p_acc = (int) std::round(gnss->h_acc / 10.0);
  gnss->ecef.vx = (int) std::round(ecef_vel.X() * 100);
  gnss->ecef.vy = (int) std::round(ecef_vel.Y() * 100);
  gnss->ecef.vz = (int) std::round(ecef_vel.Z() * 100);
  gnss->ecef.s_acc = (int) std::round(gnss_velocity_stdev_ * 100);

  gnss->rosflight_timestamp = clock_micros();

  using Vec3 = ignition::math::Vector3d;
  using Vec3 = ignition::math::Vector3d;
  using Coord = gazebo::common::SphericalCoordinates::CoordinateType;

  // TODO: Do a better job of simulating the wander of GNSS
  
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  auto now_tm = std::localtime(&now_c);
  
  gnss_full->year = now_tm->tm_year + 1900;
  gnss_full->month = now_tm->tm_mon + 1;
  gnss_full->day = now_tm->tm_mday;
  gnss_full->hour = now_tm->tm_hour;
  gnss_full->min = now_tm->tm_min;
  gnss_full->sec = now_tm->tm_sec;
  gnss_full->valid = 1;

  gnss_full->lat = (int) std::round(rad2Deg(lla.X()) * 1e7);
  gnss_full->lon = (int) std::round(rad2Deg(lla.Y()) * 1e7);
  gnss_full->height = (int) std::round(lla.Z() * 1e3);
  gnss_full->height_msl = gnss_full->height; // TODO

  // For now, we have defined the Gazebo Local Frame as NWU.  This should be
  // fixed in a future commit
  gnss_full->vel_n = (int) std::round(local_vel.X() * 1e3);
  gnss_full->vel_e = (int) std::round(-local_vel.Y() * 1e3);
  gnss_full->vel_d = (int) std::round(-local_vel.Z() * 1e3);

  gnss_full->fix_type = rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_3D_FIX;
  gnss_full->time_of_week = GZ_COMPAT_GET_SIM_TIME(world_).Double() * 1000;
  gnss_full->num_sat = 15;
  // TODO
  gnss_full->t_acc = 0;
  gnss_full->nano = 0;

  gnss_full->h_acc = (int) std::round(horizontal_gnss_stdev_ * 1000.0);
  gnss_full->v_acc = (int) std::round(vertical_gnss_stdev_ * 1000.0);

  // Again, TODO switch to using ENU convention per REP
  double vn = local_vel.X();
  double ve = -local_vel.Y();
  double ground_speed = std::sqrt(vn * vn + ve * ve);
  gnss_full->g_speed = (int) std::round(ground_speed * 1000);

  double head_mot = std::atan2(ve, vn);
  gnss_full->head_mot = (int) std::round(rad2Deg(head_mot) * 1e5);
  gnss_full->p_dop = 0.0; // TODO
  gnss_full->rosflight_timestamp = clock_micros();

  return true;
}

} // namespace rosflight_sim
