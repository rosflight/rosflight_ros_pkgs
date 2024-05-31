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

#include <fstream>
#include <rclcpp/logging.hpp>
#include <rosflight_sim/sil_board.hpp>

#include <iostream>

namespace rosflight_sim
{
SILBoard::SILBoard()
    : UDPBoard()
    , random_generator_(std::chrono::system_clock::now().time_since_epoch().count())
{}

void SILBoard::init_board() { boot_time_ = world_->SimTime(); }

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
  gyro_bias_walk_stdev_ = node_->get_parameter_or<double>("gyro_bias_walk_stdev", 0.00001);

  acc_stdev_ = node_->get_parameter_or<double>("acc_stdev", 0.025);
  acc_bias_range_ = node_->get_parameter_or<double>("acc_bias_range", 0.6);
  acc_bias_walk_stdev_ = node_->get_parameter_or<double>("acc_bias_walk_stdev", 0.00001);

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

  mass_ = node_->get_parameter_or<double>("mass", 2.28);
  rho_ = node_->get_parameter_or<double>("rho", 1.225);

  // Calculate Magnetic Field Vector (for mag simulation)
  auto inclination = node_->get_parameter_or<double>("inclination", 1.14316156541);
  auto declination = node_->get_parameter_or<double>("declination", 0.198584539676);
  inertial_magnetic_field_.Z((sin(-inclination)));
  inertial_magnetic_field_.X((cos(-inclination) * cos(-declination)));
  inertial_magnetic_field_.Y((cos(-inclination) * sin(-declination)));

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

  gravity_ = world_->Gravity();

  // Initialize the Sensor Biases
  gyro_bias_.X((gyro_bias_range_ * uniform_distribution_(random_generator_)));
  gyro_bias_.Y((gyro_bias_range_ * uniform_distribution_(random_generator_)));
  gyro_bias_.Z((gyro_bias_range_ * uniform_distribution_(random_generator_)));
  acc_bias_.X((acc_bias_range_ * uniform_distribution_(random_generator_)));
  acc_bias_.Y((acc_bias_range_ * uniform_distribution_(random_generator_)));
  acc_bias_.Z((acc_bias_range_ * uniform_distribution_(random_generator_)));
  mag_bias_.X((mag_bias_range_ * uniform_distribution_(random_generator_)));
  mag_bias_.Y((mag_bias_range_ * uniform_distribution_(random_generator_)));
  mag_bias_.Z((mag_bias_range_ * uniform_distribution_(random_generator_)));
  baro_bias_ = baro_bias_range_ * uniform_distribution_(random_generator_);
  airspeed_bias_ = airspeed_bias_range_ * uniform_distribution_(random_generator_);

  prev_vel_1_ = link_->RelativeLinearVel();
  prev_vel_2_ = link_->RelativeLinearVel();
  prev_vel_3_ = link_->RelativeLinearVel();
  last_time_ = world_->SimTime();
  next_imu_update_time_us_ = 0;
}

// clock

uint32_t SILBoard::clock_millis()
{
  uint32_t millis = (uint32_t) ((world_->SimTime() - boot_time_).Double() * 1e3);
  return millis;
}

uint64_t SILBoard::clock_micros()
{
  uint64_t micros = (uint64_t) ((world_->SimTime() - boot_time_).Double() * 1e6);
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
  gyro_bias_.X((gyro_bias_range_ * uniform_distribution_(random_generator_)));
  gyro_bias_.Y((gyro_bias_range_ * uniform_distribution_(random_generator_)));
  gyro_bias_.Z((gyro_bias_range_ * uniform_distribution_(random_generator_)));
  acc_bias_.X((acc_bias_range_ * uniform_distribution_(random_generator_)));
  acc_bias_.Y((acc_bias_range_ * uniform_distribution_(random_generator_)));
  acc_bias_.Z((acc_bias_range_ * uniform_distribution_(random_generator_)));

  // Gazebo coordinates is NWU and Earth's magnetic field is defined in NED, hence the negative signs
  double inclination_ = 1.14316156541;
  double declination_ = 0.198584539676;
  inertial_magnetic_field_.Z((sin(-inclination_)));
  inertial_magnetic_field_.X((cos(-inclination_) * cos(-declination_)));
  inertial_magnetic_field_.Y((cos(-inclination_) * sin(-declination_)));

  using SC = gazebo::common::SphericalCoordinates;
  using Ang = ignition::math::Angle;
  sph_coord_.SetSurfaceType(SC::SurfaceType::EARTH_WGS84);
  sph_coord_.SetLatitudeReference(Ang(deg2Rad(origin_latitude_)));
  sph_coord_.SetLongitudeReference(Ang(deg2Rad(origin_longitude_)));
  sph_coord_.SetElevationReference(origin_altitude_);
  // Force x-axis to be north-aligned. I promise, I will change everything to ENU in the next commit
  sph_coord_.SetHeadingOffset(Ang(M_PI / 2.0));
}

uint16_t SILBoard::num_sensor_errors() { return 0; }

bool SILBoard::new_imu_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_imu_update_time_us_) {
    next_imu_update_time_us_ = now_us + imu_update_period_us_;
    return true;
  } else {
    return false;
  }
}

bool SILBoard::imu_read(float accel[3], float * temperature, float gyro[3], uint64_t * time_us)
{
  ignition::math::Quaterniond q_I_NWU = link_->WorldPose().Rot();
  ignition::math::Vector3d current_vel = link_->RelativeLinearVel();
  ignition::math::Vector3d y_acc;
  ignition::math::Pose3d local_pose = link_->WorldPose();

  // this is James's egregious hack to overcome wild imu while sitting on the ground
  if (current_vel.Length() < 0.05) {
    y_acc = q_I_NWU.RotateVectorReverse(-gravity_);
  } else if (local_pose.Z() < 0.5) {
    y_acc = q_I_NWU.RotateVectorReverse(link_->WorldLinearAccel() - gravity_);
  } else {
    y_acc.Set(f_x/mass_, -f_y/mass_, -f_z/mass_);
  }

  // Apply normal noise (only if armed, because most of the noise comes from motors
  if (motors_spinning()) {
    y_acc.X((y_acc.X() + acc_stdev_ * normal_distribution_(random_generator_)));
    y_acc.Y((y_acc.Y() + acc_stdev_ * normal_distribution_(random_generator_)));
    y_acc.Z((y_acc.Z() + acc_stdev_ * normal_distribution_(random_generator_)));
  }

  // Perform Random Walk for biases
  acc_bias_.X((acc_bias_.X() + acc_bias_walk_stdev_ * normal_distribution_(random_generator_)));
  acc_bias_.Y((acc_bias_.Y() + acc_bias_walk_stdev_ * normal_distribution_(random_generator_)));
  acc_bias_.Z((acc_bias_.Z() + acc_bias_walk_stdev_ * normal_distribution_(random_generator_)));

  // Add constant Bias to measurement
  y_acc.X((y_acc.X() + acc_bias_.X()));
  y_acc.Y((y_acc.Y() + acc_bias_.Y()));
  y_acc.Z((y_acc.Z() + acc_bias_.Z()));

  // Convert to NED for output
  accel[0] = y_acc.X();
  accel[1] = (float) -y_acc.Y();
  accel[2] = (float) -y_acc.Z();

  ignition::math::Vector3d y_gyro = link_->RelativeAngularVel();

  // Normal Noise from motors
  if (motors_spinning()) {
    y_gyro.X((y_gyro.X() + gyro_stdev_ * normal_distribution_(random_generator_)));
    y_gyro.Y((y_gyro.Y() + gyro_stdev_ * normal_distribution_(random_generator_)));
    y_gyro.Z((y_gyro.Z() + gyro_stdev_ * normal_distribution_(random_generator_)));
  }

  // Random Walk for bias
  gyro_bias_.X((gyro_bias_.X() + gyro_bias_walk_stdev_ * normal_distribution_(random_generator_)));
  gyro_bias_.Y((gyro_bias_.Y() + gyro_bias_walk_stdev_ * normal_distribution_(random_generator_)));
  gyro_bias_.Z((gyro_bias_.Z() + gyro_bias_walk_stdev_ * normal_distribution_(random_generator_)));

  // Apply Constant Bias
  y_gyro.X((y_gyro.X() + gyro_bias_.X()));
  y_gyro.Y((y_gyro.Y() + gyro_bias_.Y()));
  y_gyro.Z((y_gyro.Z() + gyro_bias_.Z()));

  // Convert to NED for output
  gyro[0] = y_gyro.X();
  gyro[1] = (float) -y_gyro.Y();
  gyro[2] = (float) -y_gyro.Z();

  (*temperature) = 27.0 + 273.15;
  (*time_us) = clock_micros();
  return true;
}

void SILBoard::imu_not_responding_error()
{
  RCLCPP_ERROR(node_->get_logger(), "[gazebo_rosflight_sil] imu not responding");
}

void SILBoard::mag_read(float mag[3])
{
  ignition::math::Pose3d I_to_B = link_->WorldPose();
  ignition::math::Vector3d noise;
  noise.X((mag_stdev_ * normal_distribution_(random_generator_)));
  noise.Y((mag_stdev_ * normal_distribution_(random_generator_)));
  noise.Z((mag_stdev_ * normal_distribution_(random_generator_)));

  // Random Walk for bias
  mag_bias_.X((mag_bias_.X() + mag_bias_walk_stdev_ * normal_distribution_(random_generator_)));
  mag_bias_.Y((mag_bias_.Y() + mag_bias_walk_stdev_ * normal_distribution_(random_generator_)));
  mag_bias_.Z((mag_bias_.Z() + mag_bias_walk_stdev_ * normal_distribution_(random_generator_)));

  // combine parts to create a measurement
  ignition::math::Vector3d y_mag = I_to_B.Rot().RotateVectorReverse(inertial_magnetic_field_) + mag_bias_ + noise;

  // Convert measurement to NED
  mag[0] = y_mag.X();
  mag[1] = (float) -y_mag.Y();
  mag[2] = (float) -y_mag.Z();
}

bool SILBoard::mag_present() { return true; }

bool SILBoard::baro_present() { return true; }

void SILBoard::baro_read(float * pressure, float * temperature)
{
  // pull z measurement out of Gazebo
  ignition::math::Pose3d current_state_NWU = link_->WorldPose();

  // Invert measurement model for pressure and temperature
  double alt = current_state_NWU.Pos().Z() + origin_altitude_;

  // Convert to the true pressure reading
  double y_baro = 101325.0f * (float) pow((1 - 2.25694e-5 * alt), 5.2553); // Add these parameters to the parameters.

  // Add noise
  y_baro += baro_stdev_ * normal_distribution_(random_generator_);

  // Perform random walk
  baro_bias_ += baro_bias_walk_stdev_ * normal_distribution_(random_generator_);

  // Add random walk
  y_baro += baro_bias_;

  (*pressure) = (float) y_baro;
  (*temperature) = 27.0f + 273.15f;
}

bool SILBoard::diff_pressure_present()
{
  if (mav_type_ == "fixedwing") {
    return true;
  } else {
    return false;
  }
}

void SILBoard::diff_pressure_read(float * diff_pressure, float * temperature)
{
  // Calculate Airspeed
  ignition::math::Vector3d vel = link_->RelativeLinearVel();

  double Va = vel.Length();

  // Invert Airpseed to get sensor measurement
  double y_as = rho_ * Va * Va / 2.0; // Page 130 in the UAV Book

  // Add noise
  y_as += airspeed_stdev_ * normal_distribution_(random_generator_);
  airspeed_bias_ += airspeed_bias_walk_stdev_ * normal_distribution_(random_generator_);
  y_as += airspeed_bias_;

  *diff_pressure = (float) y_as;
  *temperature = 27.0 + 273.15;
}

bool SILBoard::sonar_present() { return true; }

float SILBoard::sonar_read()
{
  ignition::math::Pose3d current_state_NWU = link_->WorldPose();
  double alt = current_state_NWU.Pos().Z();

  if (alt < sonar_min_range_) {
    return (float) sonar_min_range_;
  } else if (alt > sonar_max_range_) {
    return (float) sonar_max_range_;
  } else {
    return (float) (alt + sonar_stdev_ * normal_distribution_(random_generator_));
  }
}

bool SILBoard::battery_voltage_present() const { return true; }

float SILBoard::battery_voltage_read() const { return 15 * battery_voltage_multiplier; }

void SILBoard::battery_voltage_set_multiplier(double multiplier)
{
  battery_voltage_multiplier = (float) multiplier;
}

bool SILBoard::battery_current_present() const { return true; }

float SILBoard::battery_current_read() const { return 1 * battery_current_multiplier; }

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

  for (int & pwm_output : pwm_outputs_) { pwm_output = 1000; }

  rc_sub_ = node_->create_subscription<rosflight_msgs::msg::RCRaw>(
    "RC", 1, std::bind(&SILBoard::RC_callback, this, std::placeholders::_1));
  forces_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/forces_and_moments", 1, std::bind(&SILBoard::forces_callback, this, std::placeholders::_1));
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
  if (channel == 2) { return 0.0; }

  return 0.5;
}

void SILBoard::pwm_write(uint8_t channel, float value)
{
  pwm_outputs_[channel] = 1000 + (uint16_t) (1000 * value);
}
void SILBoard::pwm_disable()
{
  for (int i = 0; i < 14; i++) { pwm_write(i, 0); }
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
  if (len < BACKUP_SRAM_SIZE) { memcpy(backup_memory_, src, len); }
}

void SILBoard::backup_memory_clear(size_t len)
{
  if (len < BACKUP_SRAM_SIZE) { memset(backup_memory_, 0, len); }
}

void SILBoard::RC_callback(const rosflight_msgs::msg::RCRaw & msg)
{
  rc_received_ = true;
  last_rc_message_ = node_->get_clock()->now();
  latestRC_ = msg;
}

bool SILBoard::gnss_present() { return true; }

rosflight_firmware::GNSSData SILBoard::gnss_read()
{
  rosflight_firmware::GNSSData out;
  using Vec3 = ignition::math::Vector3d;
  using Coord = gazebo::common::SphericalCoordinates::CoordinateType;

  ignition::math::Pose3d local_pose = link_->WorldPose();
  Vec3 pos_noise(horizontal_gps_stdev_ * normal_distribution_(random_generator_),
                 horizontal_gps_stdev_ * normal_distribution_(random_generator_),
                 vertical_gps_stdev_ * normal_distribution_(random_generator_));
  Vec3 local_pos = local_pose.Pos() + pos_noise;

  Vec3 local_vel = link_->WorldLinearVel();
  Vec3 vel_noise(gps_velocity_stdev_ * normal_distribution_(random_generator_),
                 gps_velocity_stdev_ * normal_distribution_(random_generator_),
                 gps_velocity_stdev_ * normal_distribution_(random_generator_));
  local_vel += vel_noise;

  Vec3 ecef_pos = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::ECEF);
  Vec3 ecef_vel = sph_coord_.VelocityTransform(local_vel, Coord::LOCAL, Coord::ECEF);
  Vec3 lla = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::SPHERICAL);

  out.lat = (int) std::round(rad2Deg(lla.X()) * 1e7);
  out.lon = (int) std::round(rad2Deg(lla.Y()) * 1e7);
  out.height = (int) std::round(lla.Z() * 1e3);

  // For now, we have defined the Gazebo Local Frame as NWU.  This should be fixed in a future
  // commit
  out.vel_n = (int) std::round(local_vel.X() * 1e3);
  out.vel_e = (int) std::round(-local_vel.Y() * 1e3);
  out.vel_d = (int) std::round(-local_vel.Z() * 1e3);

  out.fix_type = rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_FIX;
  out.time_of_week = world_->SimTime().Double() * 1000;
  out.time = world_->SimTime().Double();
  out.nanos =
    (uint64_t) std::round((world_->SimTime().Double() - (double) out.time) * 1e9);

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

  return out;
}

bool SILBoard::gnss_has_new_data() { return true; }

rosflight_firmware::GNSSFull SILBoard::gnss_full_read()
{
  rosflight_firmware::GNSSFull out;
  using Vec3 = ignition::math::Vector3d;
  using Vec3 = ignition::math::Vector3d;
  using Coord = gazebo::common::SphericalCoordinates::CoordinateType;

  ignition::math::Pose3d local_pose = link_->WorldPose();
  Vec3 pos_noise(horizontal_gps_stdev_ * normal_distribution_(random_generator_),
                 horizontal_gps_stdev_ * normal_distribution_(random_generator_),
                 vertical_gps_stdev_ * normal_distribution_(random_generator_));
  Vec3 local_pos = local_pose.Pos() + pos_noise;

  Vec3 local_vel = link_->WorldLinearVel();
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
  out.time_of_week = world_->SimTime().Double() * 1000;
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

  return out;
}

} // namespace rosflight_sim
