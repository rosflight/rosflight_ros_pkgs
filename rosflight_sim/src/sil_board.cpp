/*
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
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

#include <rosflight_sim/sil_board.hpp>
#include <fstream>

#include <iostream>

namespace rosflight_sim
{
SIL_Board::SIL_Board() :
  rosflight_firmware::UDPBoard(),
  random_generator_(std::chrono::system_clock::now().time_since_epoch().count()) {}

void SIL_Board::init_board()
{
  boot_time_ = GZ_COMPAT_GET_SIM_TIME(world_);
}

constexpr double rad2Deg(double x)
{
  return 180.0 / M_PI * x;
}
constexpr double deg2Rad(double x)
{
  return M_PI / 180.0 * x;
}

void SIL_Board::gazebo_setup(gazebo::physics::LinkPtr link,
                             gazebo::physics::WorldPtr world,
                             gazebo::physics::ModelPtr model,
                             rclcpp::Node::SharedPtr node,
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
        << bind_host << ":"
        << bind_port << "\n";

  // Get Sensor Parameters
  gyro_stdev_ = node_->get_parameter_or<double>("gyro_stdev", 0.00226);
  gyro_bias_range_ = node_->get_parameter_or<double>("gyro_bias_range", 0.25);
  gyro_bias_walk_stdev_ = node_->get_parameter_or<double>("gyro_bias_walk_stdev", 0.00001);

  acc_stdev_ = node_->get_parameter_or<double>("acc_stdev", 0.025);
  acc_bias_range_ = node_->get_parameter_or<double>("acc_bias_range", 0.6);
  acc_bias_walk_stdev_ = node_->get_parameter_or<double>("acc_bias_walk_stdev", 0.00001);

  // TODO: Update airspeed and mag values with empirically-derived data
  mag_stdev_ = node_->get_parameter_or<double>("mag_stdev", 0.10);
  mag_bias_range_ = node_->get_parameter_or<double>("mag_bias_range", 0.10);
  mag_bias_walk_stdev_ = node_->get_parameter_or<double>("mag_bias_walk_stdev", 0.001);

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

  // Calculate Magnetic Field Vector (for mag simulation)
  auto inclination = node_->get_parameter_or<double>("inclination", 1.14316156541);
  auto declination = node_->get_parameter_or<double>("declination", 0.198584539676);
  GZ_COMPAT_SET_Z(inertial_magnetic_field_, sin(-inclination));
  GZ_COMPAT_SET_X(inertial_magnetic_field_, cos(-inclination) * cos(-declination));
  GZ_COMPAT_SET_Y(inertial_magnetic_field_, cos(-inclination) * sin(-declination));

  // Get the desired altitude at the ground (for baro and LLA)

  origin_altitude_ = node_->get_parameter_or<double>("origin_altitude", 1387.0);
  origin_latitude_ = node_->get_parameter_or<double>("origin_latitude", 40.2463724);
  origin_longitude_ = node_->get_parameter_or<double>("origin_longitude", -111.6474138);

  horizontal_gps_stdev_ = node_->get_parameter_or<double>("horizontal_gps_stdev", 1.0);
  vertical_gps_stdev_ = node_->get_parameter_or<double>("vertical_gps_stdev", 3.0);
  gps_velocity_stdev_ = node_->get_parameter_or<double>("gps_velocity_stdev", 0.1);

  // Configure Noise
  normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  uniform_distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);

  gravity_ = GZ_COMPAT_GET_GRAVITY(world_);

  // Initialize the Sensor Biases
  GZ_COMPAT_SET_X(gyro_bias_, gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(gyro_bias_, gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(gyro_bias_, gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_X(acc_bias_, acc_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(acc_bias_, acc_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(acc_bias_, acc_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_X(mag_bias_, mag_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(mag_bias_, mag_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(mag_bias_, mag_bias_range_ * uniform_distribution_(random_generator_));
  baro_bias_ = baro_bias_range_ * uniform_distribution_(random_generator_);
  airspeed_bias_ = airspeed_bias_range_ * uniform_distribution_(random_generator_);

  prev_vel_1_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  prev_vel_2_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  prev_vel_3_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  last_time_ = GZ_COMPAT_GET_SIM_TIME(world_);
  next_imu_update_time_us_ = 0;
}

void SIL_Board::board_reset(bool bootloader) {}

// clock

uint32_t SIL_Board::clock_millis()
{
  uint32_t millis = (uint32_t) ((GZ_COMPAT_GET_SIM_TIME(world_) - boot_time_).Double() * 1e3);
  return millis;
}

uint64_t SIL_Board::clock_micros()
{
  uint64_t micros = (uint64_t) ((GZ_COMPAT_GET_SIM_TIME(world_) - boot_time_).Double() * 1e6);
  return micros;
}

void SIL_Board::clock_delay(uint32_t milliseconds) {}

// sensors
/// TODO these sensors have noise, no bias
/// noise params are hard coded
void SIL_Board::sensors_init()
{
  // Initialize the Biases
  GZ_COMPAT_SET_X(gyro_bias_, gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(gyro_bias_, gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(gyro_bias_, gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_X(acc_bias_, acc_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(acc_bias_, acc_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(acc_bias_, acc_bias_range_ * uniform_distribution_(random_generator_));

  // Gazebo coordinates is NWU and Earth's magnetic field is defined in NED, hence the negative signs
  double inclination_ = 1.14316156541;
  double declination_ = 0.198584539676;
  GZ_COMPAT_SET_Z(inertial_magnetic_field_, sin(-inclination_));
  GZ_COMPAT_SET_X(inertial_magnetic_field_, cos(-inclination_) * cos(-declination_));
  GZ_COMPAT_SET_Y(inertial_magnetic_field_, cos(-inclination_) * sin(-declination_));

#if GAZEBO_MAJOR_VERSION >= 9
  using SC = gazebo::common::SphericalCoordinates;
  using Ang = ignition::math::Angle;
  sph_coord_.SetSurfaceType(SC::SurfaceType::EARTH_WGS84);
  sph_coord_.SetLatitudeReference(Ang(deg2Rad(origin_latitude_)));
  sph_coord_.SetLongitudeReference(Ang(deg2Rad(origin_longitude_)));
  sph_coord_.SetElevationReference(origin_altitude_);
  // Force x-axis to be north-aligned. I promise, I will change everything to ENU in the next commit
  sph_coord_.SetHeadingOffset(Ang(-M_PI / 2.0));
#endif
}

uint16_t SIL_Board::num_sensor_errors()
{
  return 0;
}

bool SIL_Board::new_imu_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_imu_update_time_us_) {
    next_imu_update_time_us_ = now_us + imu_update_period_us_;
    return true;
  } else {
    return false;
  }
}

bool SIL_Board::imu_read(float accel[3], float * temperature, float gyro[3], uint64_t * time_us)
{
  GazeboQuaternion q_I_NWU = GZ_COMPAT_GET_ROT(GZ_COMPAT_GET_WORLD_POSE(link_));
  GazeboVector current_vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  GazeboVector y_acc;

  // this is James' egregious hack to overcome wild imu while sitting on the ground
  if (GZ_COMPAT_GET_LENGTH(current_vel) < 0.05) {
    y_acc = q_I_NWU.RotateVectorReverse(-gravity_);
  } else {
    y_acc = q_I_NWU.RotateVectorReverse(GZ_COMPAT_GET_WORLD_LINEAR_ACCEL(link_) - gravity_);
  }

  // Apply normal noise (only if armed, because most of the noise comes from motors
  if (motors_spinning()) {
    GZ_COMPAT_SET_X(y_acc,
                    GZ_COMPAT_GET_X(y_acc) + acc_stdev_ * normal_distribution_(random_generator_));
    GZ_COMPAT_SET_Y(y_acc,
                    GZ_COMPAT_GET_Y(y_acc) + acc_stdev_ * normal_distribution_(random_generator_));
    GZ_COMPAT_SET_Z(y_acc,
                    GZ_COMPAT_GET_Z(y_acc) + acc_stdev_ * normal_distribution_(random_generator_));
  }

  // Perform Random Walk for biases
  GZ_COMPAT_SET_X(acc_bias_,
                  GZ_COMPAT_GET_X(acc_bias_)
                    + acc_bias_walk_stdev_ * normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(acc_bias_,
                  GZ_COMPAT_GET_Y(acc_bias_)
                    + acc_bias_walk_stdev_ * normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(acc_bias_,
                  GZ_COMPAT_GET_Z(acc_bias_)
                    + acc_bias_walk_stdev_ * normal_distribution_(random_generator_));

  // Add constant Bias to measurement
  GZ_COMPAT_SET_X(y_acc, GZ_COMPAT_GET_X(y_acc) + GZ_COMPAT_GET_X(acc_bias_));
  GZ_COMPAT_SET_Y(y_acc, GZ_COMPAT_GET_Y(y_acc) + GZ_COMPAT_GET_Y(acc_bias_));
  GZ_COMPAT_SET_Z(y_acc, GZ_COMPAT_GET_Z(y_acc) + GZ_COMPAT_GET_Z(acc_bias_));

  // Convert to NED for output
  accel[0] = GZ_COMPAT_GET_X(y_acc);
  accel[1] = (float) -GZ_COMPAT_GET_Y(y_acc);
  accel[2] = (float) -GZ_COMPAT_GET_Z(y_acc);

  GazeboVector y_gyro = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);

  // Normal Noise from motors
  if (motors_spinning()) {
    GZ_COMPAT_SET_X(y_gyro,
                    GZ_COMPAT_GET_X(y_gyro)
                      + gyro_stdev_ * normal_distribution_(random_generator_));
    GZ_COMPAT_SET_Y(y_gyro,
                    GZ_COMPAT_GET_Y(y_gyro)
                      + gyro_stdev_ * normal_distribution_(random_generator_));
    GZ_COMPAT_SET_Z(y_gyro,
                    GZ_COMPAT_GET_Z(y_gyro)
                      + gyro_stdev_ * normal_distribution_(random_generator_));
  }

  // Random Walk for bias
  GZ_COMPAT_SET_X(gyro_bias_,
                  GZ_COMPAT_GET_X(gyro_bias_)
                    + gyro_bias_walk_stdev_ * normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(gyro_bias_,
                  GZ_COMPAT_GET_Y(gyro_bias_)
                    + gyro_bias_walk_stdev_ * normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(gyro_bias_,
                  GZ_COMPAT_GET_Z(gyro_bias_)
                    + gyro_bias_walk_stdev_ * normal_distribution_(random_generator_));

  // Apply Constant Bias
  GZ_COMPAT_SET_X(y_gyro, GZ_COMPAT_GET_X(y_gyro) + GZ_COMPAT_GET_X(gyro_bias_));
  GZ_COMPAT_SET_Y(y_gyro, GZ_COMPAT_GET_Y(y_gyro) + GZ_COMPAT_GET_Y(gyro_bias_));
  GZ_COMPAT_SET_Z(y_gyro, GZ_COMPAT_GET_Z(y_gyro) + GZ_COMPAT_GET_Z(gyro_bias_));

  // Convert to NED for output
  gyro[0] = GZ_COMPAT_GET_X(y_gyro);
  gyro[1] = (float) -GZ_COMPAT_GET_Y(y_gyro);
  gyro[2] = (float) -GZ_COMPAT_GET_Z(y_gyro);

  (*temperature) = 27.0;
  (*time_us) = clock_micros();
  return true;
}

void SIL_Board::imu_not_responding_error()
{
  RCLCPP_ERROR(node_->get_logger(), "[gazebo_rosflight_sil] imu not responding");
}

void SIL_Board::mag_read(float mag[3])
{
  GazeboPose I_to_B = GZ_COMPAT_GET_WORLD_POSE(link_);
  GazeboVector noise;
  GZ_COMPAT_SET_X(noise, mag_stdev_ * normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(noise, mag_stdev_ * normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(noise, mag_stdev_ * normal_distribution_(random_generator_));

  // Random Walk for bias
  GZ_COMPAT_SET_X(mag_bias_,
                  GZ_COMPAT_GET_X(mag_bias_)
                    + mag_bias_walk_stdev_ * normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(mag_bias_,
                  GZ_COMPAT_GET_Y(mag_bias_)
                    + mag_bias_walk_stdev_ * normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(mag_bias_,
                  GZ_COMPAT_GET_Z(mag_bias_)
                    + mag_bias_walk_stdev_ * normal_distribution_(random_generator_));

  // combine parts to create a measurement
  GazeboVector y_mag =
    GZ_COMPAT_GET_ROT(I_to_B).RotateVectorReverse(inertial_magnetic_field_) + mag_bias_ + noise;

  // Convert measurement to NED
  mag[0] = GZ_COMPAT_GET_X(y_mag);
  mag[1] = (float) -GZ_COMPAT_GET_Y(y_mag);
  mag[2] = (float) -GZ_COMPAT_GET_Z(y_mag);
}

bool SIL_Board::mag_present()
{
  return true;
}

bool SIL_Board::baro_present()
{
  return true;
}

void SIL_Board::baro_read(float * pressure, float * temperature)
{
  // pull z measurement out of Gazebo
  GazeboPose current_state_NWU = GZ_COMPAT_GET_WORLD_POSE(link_);

  // Invert measurement model for pressure and temperature
  double alt = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(current_state_NWU)) + origin_altitude_;

  // Convert to the true pressure reading
  double y_baro = 101325.0f * (float) pow((1 - 2.25694e-5 * alt), 5.2553);

  // Add noise
  y_baro += baro_stdev_ * normal_distribution_(random_generator_);

  // Perform random walk
  baro_bias_ += baro_bias_walk_stdev_ * normal_distribution_(random_generator_);

  // Add random walk
  y_baro += baro_bias_;

  (*pressure) = (float) y_baro;
  (*temperature) = 27.0f;
}

bool SIL_Board::diff_pressure_present()
{
  if (mav_type_ == "fixedwing") {
    return true;
  } else {
    return false;
  }
}

void SIL_Board::diff_pressure_read(float * diff_pressure, float * temperature)
{
  static double rho_ = 1.225;
  // Calculate Airspeed
  GazeboVector vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);

  double Va = GZ_COMPAT_GET_LENGTH(vel);

  // Invert Airpseed to get sensor measurement
  double y_as = rho_ * Va * Va / 2.0; // Page 130 in the UAV Book

  // Add noise
  y_as += airspeed_stdev_ * normal_distribution_(random_generator_);
  airspeed_bias_ += airspeed_bias_walk_stdev_ * normal_distribution_(random_generator_);
  y_as += airspeed_bias_;

  *diff_pressure = (float) y_as;
  *temperature = 27.0;
}

bool SIL_Board::sonar_present()
{
  return true;
}

float SIL_Board::sonar_read()
{
  GazeboPose current_state_NWU = GZ_COMPAT_GET_WORLD_POSE(link_);
  double alt = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(current_state_NWU));

  if (alt < sonar_min_range_) {
    return (float) sonar_min_range_;
  } else if (alt > sonar_max_range_) {
    return (float) sonar_max_range_;
  } else {
    return (float) (alt + sonar_stdev_ * normal_distribution_(random_generator_));
  }
}

bool SIL_Board::battery_voltage_present() const
{
  return true;
}

float SIL_Board::battery_voltage_read() const
{
  return 15 * battery_voltage_multiplier;
}

void SIL_Board::battery_voltage_set_multiplier(double multiplier)
{
  battery_voltage_multiplier = (float) multiplier;
}

bool SIL_Board::battery_current_present() const
{
  return true;
}

float SIL_Board::battery_current_read() const
{
  return 1 * battery_current_multiplier;
}

void SIL_Board::battery_current_set_multiplier(double multiplier)
{
  battery_current_multiplier = (float) multiplier;
}

// PWM
void SIL_Board::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  rc_received_ = false;
  latestRC_.values[0] = 1500; // x
  latestRC_.values[1] = 1500; // y
  latestRC_.values[3] = 1500; // z
  latestRC_.values[2] = 1000; // F
  latestRC_.values[4] = 1000; // attitude override
  latestRC_.values[5] = 1000; // arm

  for (int &pwm_output : pwm_outputs_) { pwm_output = 1000; }

  rc_sub_ = node_->create_subscription<rosflight_msgs::msg::RCRaw>(
    "RC", 1, std::bind(&SIL_Board::RCCallback, this, std::placeholders::_1));
}

float SIL_Board::rc_read(uint8_t channel)
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

void SIL_Board::pwm_write(uint8_t channel, float value)
{
  pwm_outputs_[channel] = 1000 + (uint16_t) (1000 * value);
}
void SIL_Board::pwm_disable()
{
  for (int i = 0; i < 14; i++) { pwm_write(i, 0); }
}

bool SIL_Board::rc_lost()
{
  return !rc_received_;
}

void SIL_Board::rc_init(rc_type_t rc_type) {}

// non-volatile memory
void SIL_Board::memory_init() {}

bool SIL_Board::memory_read(void * dest, size_t len)
{
  std::string directory = "rosflight_memory" + std::string(node_->get_namespace());
  std::ifstream memory_file;
  memory_file.open(directory + "/mem.bin", std::ios::binary);

  if (!memory_file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Unable to load rosflight memory file %s/mem.bin",
                 directory.c_str());
    return false;
  }

  memory_file.read((char *) dest, (long) len);
  memory_file.close();
  return true;
}

bool SIL_Board::memory_write(const void * src, size_t len)
{
  std::string directory = "rosflight_memory" + std::string(node_->get_namespace());
  std::string mkdir_command = "mkdir -p " + directory;
  const int dir_err = system(mkdir_command.c_str());

  if (dir_err == -1) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Unable to write rosflight memory file %s/mem.bin",
                 directory.c_str());
    return false;
  }

  std::ofstream memory_file;
  memory_file.open(directory + "/mem.bin", std::ios::binary);
  memory_file.write((char *) src, (long) len);
  memory_file.close();
  return true;
}

bool SIL_Board::motors_spinning()
{
  if (pwm_outputs_[2] > 1100) {
    return true;
  } else {
    return false;
  }
}

// LED

void SIL_Board::led0_on() {}
void SIL_Board::led0_off() {}
void SIL_Board::led0_toggle() {}

void SIL_Board::led1_on() {}
void SIL_Board::led1_off() {}
void SIL_Board::led1_toggle() {}

void SIL_Board::backup_memory_init() {}

bool SIL_Board::backup_memory_read(void * dest, size_t len)
{
  if (len <= BACKUP_SRAM_SIZE) {
    memcpy(dest, backup_memory_, len);
    return true;
  } else {
    return false;
  }
}

void SIL_Board::backup_memory_write(const void * src, size_t len)
{
  if (len < BACKUP_SRAM_SIZE) {
    memcpy(backup_memory_, src, len);
  }
}

void SIL_Board::backup_memory_clear(size_t len)
{
  if (len < BACKUP_SRAM_SIZE) {
    memset(backup_memory_, 0, len);
  }
}

void SIL_Board::RCCallback(const rosflight_msgs::msg::RCRaw &msg)
{
  rc_received_ = true;
  last_rc_message_ = node_->get_clock()->now();
  latestRC_ = msg;
}

bool SIL_Board::gnss_present()
{
  return GAZEBO_MAJOR_VERSION >= 9;
}
void SIL_Board::gnss_update() {}

rosflight_firmware::GNSSData SIL_Board::gnss_read()
{
  rosflight_firmware::GNSSData out;
#if GAZEBO_MAJOR_VERSION >= 9
  using Vec3 = ignition::math::Vector3d;
  using Coord = gazebo::common::SphericalCoordinates::CoordinateType;

  GazeboPose local_pose = GZ_COMPAT_GET_WORLD_POSE(link_);
  Vec3 pos_noise(horizontal_gps_stdev_ * normal_distribution_(random_generator_),
                 horizontal_gps_stdev_ * normal_distribution_(random_generator_),
                 vertical_gps_stdev_ * normal_distribution_(random_generator_));
  Vec3 local_pos = GZ_COMPAT_GET_POS(local_pose) + pos_noise;

  Vec3 local_vel = GZ_COMPAT_GET_WORLD_LINEAR_VEL(link_);
  Vec3 vel_noise(gps_velocity_stdev_ * normal_distribution_(random_generator_),
                 gps_velocity_stdev_ * normal_distribution_(random_generator_),
                 gps_velocity_stdev_ * normal_distribution_(random_generator_));
  local_vel += vel_noise;

  Vec3 ecef_pos = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::ECEF);
  Vec3 ecef_vel = sph_coord_.VelocityTransform(local_vel, Coord::LOCAL, Coord::ECEF);
  Vec3 lla = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::SPHERICAL);

  out.lat = (int) std::round(rad2Deg(lla.X()) * 1e7);
  out.lon = (int) std::round(rad2Deg(lla.Y()) * 1e7);
  out.height = (int) std::round(rad2Deg(lla.Z()) * 1e3);

  // For now, we have defined the Gazebo Local Frame as NWU.  This should be fixed in a future
  // commit
  out.vel_n = (int) std::round(local_vel.X() * 1e3);
  out.vel_e = (int) std::round(-local_vel.Y() * 1e3);
  out.vel_d = (int) std::round(-local_vel.Z() * 1e3);

  out.fix_type = rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_FIX;
  out.time_of_week = GZ_COMPAT_GET_SIM_TIME(world_).Double() * 1000;
  out.time = GZ_COMPAT_GET_SIM_TIME(world_).Double();
  out.nanos =
    (uint64_t) std::round((GZ_COMPAT_GET_SIM_TIME(world_).Double() - (double) out.time) * 1e9);

  out.h_acc = (int) std::round(horizontal_gps_stdev_ * 1000.0);
  out.v_acc = (int) std::round(vertical_gps_stdev_ * 1000.0);

  out.ecef.x = (int) std::round(ecef_pos.X() * 100);
  out.ecef.y = (int) std::round(ecef_pos.Y() * 100);
  out.ecef.z = (int) std::round(ecef_pos.Z() * 100);
  out.ecef.p_acc = (int) std::round(out.h_acc / 10.0);
  out.ecef.vx = (int) std::round(ecef_vel.X() * 100);
  out.ecef.vy = (int) std::round(ecef_vel.Y() * 100);
  out.ecef.vz = (int) std::round(ecef_vel.Z() * 100);
  out.ecef.s_acc = (int) std::round(gps_velocity_stdev_ * 100);

  out.rosflight_timestamp = clock_micros();

#endif
  return out;
}

bool SIL_Board::gnss_has_new_data()
{
  return GAZEBO_MAJOR_VERSION >= 9;
}
rosflight_firmware::GNSSFull SIL_Board::gnss_full_read()
{
  rosflight_firmware::GNSSFull out;
#if GAZEBO_MAJOR_VERSION >= 9
  using Vec3 = ignition::math::Vector3d;
  using Vec3 = ignition::math::Vector3d;
  using Coord = gazebo::common::SphericalCoordinates::CoordinateType;

  GazeboPose local_pose = GZ_COMPAT_GET_WORLD_POSE(link_);
  Vec3 pos_noise(horizontal_gps_stdev_ * normal_distribution_(random_generator_),
                 horizontal_gps_stdev_ * normal_distribution_(random_generator_),
                 vertical_gps_stdev_ * normal_distribution_(random_generator_));
  Vec3 local_pos = GZ_COMPAT_GET_POS(local_pose) + pos_noise;

  Vec3 local_vel = GZ_COMPAT_GET_WORLD_LINEAR_VEL(link_);
  Vec3 vel_noise(gps_velocity_stdev_ * normal_distribution_(random_generator_),
                 gps_velocity_stdev_ * normal_distribution_(random_generator_),
                 gps_velocity_stdev_ * normal_distribution_(random_generator_));
  local_vel += vel_noise;

  // TODO: Do a better job of simulating the wander of GPS

  Vec3 ecef_pos = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::ECEF);
  Vec3 ecef_vel = sph_coord_.VelocityTransform(local_vel, Coord::LOCAL, Coord::ECEF);
  Vec3 lla = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::SPHERICAL);

  out.lat = (int) std::round(rad2Deg(lla.X()) * 1e7);
  out.lon = (int) std::round(rad2Deg(lla.Y()) * 1e7);
  out.height = (int) std::round(rad2Deg(lla.Z()) * 1e3);
  out.height_msl = out.height; // TODO

  // For now, we have defined the Gazebo Local Frame as NWU.  This should be
  // fixed in a future commit
  out.vel_n = (int) std::round(local_vel.X() * 1e3);
  out.vel_e = (int) std::round(-local_vel.Y() * 1e3);
  out.vel_d = (int) std::round(-local_vel.Z() * 1e3);

  out.fix_type = rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_FIX;
  out.time_of_week = GZ_COMPAT_GET_SIM_TIME(world_).Double() * 1000;
  out.num_sat = 15;
  // TODO
  out.year = 0;
  out.month = 0;
  out.day = 0;
  out.hour = 0;
  out.min = 0;
  out.sec = 0;
  out.valid = 0;
  out.t_acc = 0;
  out.nano = 0;

  out.h_acc = (int) std::round(horizontal_gps_stdev_ * 1000.0);
  out.v_acc = (int) std::round(vertical_gps_stdev_ * 1000.0);

  // Again, TODO switch to using ENU convention per REP
  double vn = local_vel.X();
  double ve = -local_vel.Y();
  double ground_speed = std::sqrt(vn * vn + ve * ve);
  out.g_speed = (int) std::round(ground_speed * 1000);

  double head_mot = std::atan2(ve, vn);
  out.head_mot = (int) std::round(rad2Deg(head_mot) * 1e5);
  out.p_dop = 0.0; // TODO
  out.rosflight_timestamp = clock_micros();

#endif
  return out;
}

} // namespace rosflight_sim
