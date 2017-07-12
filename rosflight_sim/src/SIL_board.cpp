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

#include "SIL_board.h"
#include <fstream>

namespace rosflight_sim {

void SIL_Board::init_board(void)
{

}

void SIL_Board::gazebo_setup(gazebo::physics::LinkPtr link, gazebo::physics::WorldPtr world, gazebo::physics::ModelPtr model, ros::NodeHandle* nh, std::string mav_type)
{
  link_ = link;
  world_ = world;
  model_ = model;
  nh_ = nh;
  mav_type_ = mav_type;

  // Get Parameters
  ground_altitude_ = nh->param<double>("ground_altitude", 1387.0);

  gyro_stdev_ = nh->param<double>("gyro_stdev", 0.13);
  gyro_bias_range_ = nh->param<double>("gyro_bias_range", 0.15);
  gyro_bias_walk_stdev_ = nh->param<double>("gyro_bias_walk_stdev", 0.001);

  acc_stdev_ = nh->param<double>("acc_stdev", 1.15);
  acc_bias_range_ = nh->param<double>("acc_bias_range", 0.15);
  acc_bias_walk_stdev_ = nh->param<double>("acc_bias_walk_stdev", 0.001);

  mag_stdev_ = nh->param<double>("mag_stdev", 1.15);
  mag_bias_range_ = nh->param<double>("mag_bias_range", 0.15);
  mag_bias_walk_stdev_ = nh->param<double>("mag_bias_walk_stdev", 0.001);

  baro_stdev_ = nh->param<double>("baro_stdev", 1.15);
  baro_bias_range_ = nh->param<double>("baro_bias_range", 0.15);
  baro_bias_walk_stdev_ = nh->param<double>("baro_bias_walk_stdev", 0.001);

  airspeed_stdev_ = nh_->param<double>("airspeed_stdev", 1.15);
  airspeed_bias_range_ = nh_->param<double>("airspeed_bias_range", 0.15);
  airspeed_bias_walk_stdev_ = nh_->param<double>("airspeed_bias_walk_stdev", 0.001);

  sonar_stdev_ = nh_->param<double>("sonar_stdev", 1.15);
  sonar_min_range_ = nh_->param<double>("sonar_min_range", 0.25);
  sonar_max_range_ = nh_->param<double>("sonar_max_range", 8.0);

  // Configure Noise
  random_generator_= std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
  normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  uniform_distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);

  gravity_ = world_->GetPhysicsEngine()->GetGravity();

  // Initialize the Biases
  gyro_bias_.x = gyro_bias_range_*uniform_distribution_(random_generator_);
  gyro_bias_.y = gyro_bias_range_*uniform_distribution_(random_generator_);
  gyro_bias_.z = gyro_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_.x = acc_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_.y = acc_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_.z = acc_bias_range_*uniform_distribution_(random_generator_);
  mag_bias_.x = mag_bias_range_*uniform_distribution_(random_generator_);
  mag_bias_.y = mag_bias_range_*uniform_distribution_(random_generator_);
  mag_bias_.z = mag_bias_range_*uniform_distribution_(random_generator_);
  baro_bias_ = baro_bias_range_*uniform_distribution_(random_generator_);
  airspeed_bias_ = airspeed_bias_range_*uniform_distribution_(random_generator_);
}

void SIL_Board::board_reset(bool bootloader)
{

}

// clock

uint32_t SIL_Board::clock_millis()
{
  return (uint32_t)(world_->GetSimTime().Double()*1e3);
}

uint64_t SIL_Board::clock_micros()
{
  return (uint64_t)(world_->GetSimTime().Double()*1e6);
}

void SIL_Board::clock_delay(uint32_t milliseconds)
{
}

// serial

void SIL_Board::serial_init(uint32_t baud_rate)
{

}

void SIL_Board::serial_write(uint8_t byte)
{
}

uint16_t SIL_Board::serial_bytes_available(void)
{
}

uint8_t SIL_Board::serial_read(void)
{
}

// sensors
/// TODO these sensors have noise, no bias
/// noise params are hard coded
void SIL_Board::sensors_init()
{
}

uint16_t SIL_Board::num_sensor_errors(void)
{
}


bool SIL_Board::new_imu_data()
{
  double now = world_->GetSimTime().Double();
  if (now > next_imu_update_time_)
  {
    next_imu_update_time_ += 1/imu_update_rate_;
    return true;
  }
  else
    return false;

}

void SIL_Board::imu_read_accel(float accel[3])
{
  gazebo::math::Quaternion q_I_NWU = link_->GetWorldPose().rot;
  gazebo::math::Vector3 omega_B_NWU = link_->GetRelativeAngularVel();
  gazebo::math::Vector3 uvw_B_NWU = link_->GetRelativeLinearVel();

  // y_acc = vdot - R*g + w X v
  gazebo::math::Vector3 y_acc = link_->GetRelativeLinearAccel() - q_I_NWU.RotateVectorReverse(gravity_) + omega_B_NWU.Cross(uvw_B_NWU);

  // Apply normal noise
  y_acc.x += acc_stdev_*normal_distribution_(random_generator_);
  y_acc.y += acc_stdev_*normal_distribution_(random_generator_);
  y_acc.z += acc_stdev_*normal_distribution_(random_generator_);

  // Perform Random Walk for biases
  acc_bias_.x += acc_bias_walk_stdev_*normal_distribution_(random_generator_);
  acc_bias_.y += acc_bias_walk_stdev_*normal_distribution_(random_generator_);
  acc_bias_.z += acc_bias_walk_stdev_*normal_distribution_(random_generator_);

  // Add constant Bias to measurement
  y_acc.x += acc_bias_.x;
  y_acc.y += acc_bias_.y;
  y_acc.z += acc_bias_.z;

  // Convert to NED for output
  accel[0] = y_acc.x;
  accel[1] = -y_acc.y;
  accel[2] = -y_acc.z;
}

void SIL_Board::imu_read_gyro(float gyro[3])
{
  gazebo::math::Vector3 y_gyro = link_->GetRelativeAngularVel();

  // Normal Noise
  y_gyro.x += gyro_stdev_*normal_distribution_(random_generator_);
  y_gyro.y += gyro_stdev_*normal_distribution_(random_generator_);
  y_gyro.z += gyro_stdev_*normal_distribution_(random_generator_);

  // Random Walk for bias
  gyro_bias_.x += gyro_bias_walk_stdev_*normal_distribution_(random_generator_);
  gyro_bias_.y += gyro_bias_walk_stdev_*normal_distribution_(random_generator_);
  gyro_bias_.z += gyro_bias_walk_stdev_*normal_distribution_(random_generator_);

  // Apply Constant Bias
  y_gyro.x += gyro_bias_.x;
  y_gyro.y += gyro_bias_.y;
  y_gyro.z += gyro_bias_.z;

  gyro[0] = y_gyro.x;
  gyro[1] = -y_gyro.y;
  gyro[2] = -y_gyro.z;
}

bool SIL_Board::imu_read_all(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
{
  imu_read_accel(accel);
  imu_read_gyro(gyro);
  *temperature = imu_read_temperature();
  *time_us = (uint64_t)(world_->GetSimTime().Double() * 1e6);
}

float SIL_Board::imu_read_temperature(void)
{
  return 27.0f;
}

void SIL_Board::imu_not_responding_error(void)
{
  gzerr << "[gazebo_rosflight_sil] imu not responding.\n";
}

bool SIL_Board::mag_present(void)
{
  // Gazebo coordinates is NWU and Earth's magnetic field is defined in NED, hence the negative signs
  double inclination_ = 1.14316156541;
  double declination_ = 0.198584539676;
  inertial_magnetic_field_.z = sin(-inclination_);
  inertial_magnetic_field_.x = cos(-inclination_)*cos(-declination_);
  inertial_magnetic_field_.y = cos(-inclination_)*sin(-declination_);
  return true;
}

void SIL_Board::mag_read(float mag[3])
{
  gazebo::math::Pose I_to_B = link_->GetWorldPose();
  gazebo::math::Vector3 noise;
  noise.x = 0.02*normal_distribution_(random_generator_);
  noise.y = 0.02*normal_distribution_(random_generator_);
  noise.z = 0.02*normal_distribution_(random_generator_);

  // combine parts to create a measurement
  gazebo::math::Vector3 measurement = I_to_B.rot.RotateVectorReverse(inertial_magnetic_field_);

  measurement += noise;

  // normalize measurement
  gazebo::math::Vector3 normalized = measurement.Normalize();

  mag[0] = normalized.x;
  mag[1] = -normalized.y;
  mag[2] = -normalized.z;
}

bool SIL_Board::mag_check(void)
{
  return true;
}

bool SIL_Board::baro_check()
{
  return true;
}

bool SIL_Board::baro_present(void)
{
  return true;
}

void SIL_Board::baro_read(float *altitude, float *pressure, float *temperature)
{
  // pull z measurement out of Gazebo
  gazebo::math::Pose current_state_NWU = link_->GetWorldPose();

  // Invert measurement model for pressure and temperature
  double alt = current_state_NWU.pos.z + ground_altitude_;

  // Convert to the true pressure reading
  double y_baro = 101325.0 * pow(1- (2.25577e-5 * alt), 5.25588);

  // Add noise
  y_baro += baro_stdev_*normal_distribution_(random_generator_);

  // Perform random walk
  baro_bias_ += baro_bias_walk_stdev_*normal_distribution_(random_generator_);

  // Add random walk
  y_baro += baro_bias_;

  (*pressure) = (float)y_baro;
  (*temperature) = 27.0f;
  (*altitude) = (1.0 - pow(y_baro/101325.0, 0.1902631)) * 39097.63;
}

void SIL_Board::baro_calibrate()
{
  baro_bias_ = 0.0;
}

bool SIL_Board::diff_pressure_present(void)
{
  if(mav_type_ == "fixedwing")
    return true;
  else
    return false;
}

bool SIL_Board::diff_pressure_check(void)
{
  return diff_pressure_present();
}

void SIL_Board::diff_pressure_calibrate()
{
  airspeed_bias_ = 0.0;
}

void SIL_Board::diff_pressure_set_atm(float barometric_pressure)
{
  (void)barometric_pressure;
}

void SIL_Board::diff_pressure_read(float *diff_pressure, float *temperature, float *velocity)
{
  static double rho_ = 1.225;
  // Calculate Airspeed
  gazebo::math::Vector3 uvw_B_NWU = link_->GetRelativeLinearVel();

  /// TODO: Wind is being applied in the inertial frame, not the body-fixed frame
  //  double ur = u - wind_.N;
  //  double vr = v - wind_.E;
  //  double wr = w - wind_.D;
  //  double Va = sqrt(pow(ur,2.0) + pow(vr,2.0) + pow(wr,2.0));
  double Va = uvw_B_NWU.GetLength();

  // Invert Airpseed to get sensor measurement
  double y_as = rho_*Va*Va/2.0; // Page 130 in the UAV Book

  // Add noise
  y_as += airspeed_stdev_*normal_distribution_(random_generator_);
  airspeed_bias_ += airspeed_bias_walk_stdev_*normal_distribution_(random_generator_);
  y_as += airspeed_bias_;

  *diff_pressure = y_as;
  *temperature = 27.0;
  *velocity = sqrt(2*y_as/rho_);
}

bool SIL_Board::sonar_present(void)
{
  return true;
}

bool SIL_Board::sonar_check(void)
{
  return true;
}

float SIL_Board::sonar_read(void)
{
  gazebo::math::Pose current_state_NWU = link_->GetWorldPose();
  double alt = current_state_NWU.pos.z;

  if (alt < sonar_min_range_)
  {
    return sonar_min_range_;
  }
  else if (alt > sonar_max_range_)
  {
    return sonar_max_range_;
  }
  else
    return alt + sonar_stdev_*normal_distribution_(random_generator_);
}

// PWM
void SIL_Board::pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{
  rc_sub_ = nh_->subscribe("RC", 1, &SIL_Board::RCCallback, this);
}

void SIL_Board::RCCallback(const rosflight_msgs::RCRaw& msg)
{
  latestRC_ = msg;
}

uint16_t SIL_Board::pwm_read(uint8_t channel)
{
  if(rc_sub_.getNumPublishers() > 0)
  {
    return latestRC_.values[channel];
  }

  //no publishers, set throttle low and center everything else
  if(channel == 2)
    return 1000;

  return 1500;
}

void SIL_Board::pwm_write(uint8_t channel, uint16_t value)
{
  pwm_outputs_[channel] = value;
}

bool SIL_Board::pwm_lost()
{
  return false;
}

// non-volatile memory
void SIL_Board::memory_init(void) {}

bool SIL_Board::memory_read(void * dest, size_t len)
{
  std::string directory = "rosflight_memory/" + nh_->getNamespace();
  std::ifstream memory_file;
  memory_file.open(directory + "/mem.bin", std::ios::binary);

  if(!memory_file.is_open())
  {
    ROS_ERROR("Unable to load rosflight memory file %s/mem.bin", directory.c_str());
    return false;
  }

  memory_file.read((char*) dest, len);
  memory_file.close();
  return true;
}

bool SIL_Board::memory_write(const void * src, size_t len)
{
  std::string directory = "rosflight_memory/" + nh_->getNamespace();
  std::string mkdir_command = "mkdir -p" + directory;
  const int dir_err = system(mkdir_command.c_str());

  if (dir_err == -1)
  {
    ROS_ERROR("Unable to write rosflight memory file %s/mem.bin", directory.c_str());
    return false;
  }

  std::ofstream memory_file;
  memory_file.open(directory + "/mem.bin", std::ios::binary);
  memory_file.write((char*) src, len);
  memory_file.close();
  return true;
}

// LED

void SIL_Board::led0_on(void) { }
void SIL_Board::led0_off(void) { }
void SIL_Board::led0_toggle(void) { }

void SIL_Board::led1_on(void) { }
void SIL_Board::led1_off(void) { }
void SIL_Board::led1_toggle(void) { }

}
