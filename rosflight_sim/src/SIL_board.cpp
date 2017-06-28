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

namespace rosflight {

void SIL_Board::init_board(void)
{

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
/// TODO these are perfect sensors as of right now
void SIL_Board::sensors_init()
{
}

uint16_t SIL_Board::num_sensor_errors(void)
{
}

void SIL_Board::imu_register_callback(void (*callback)(void))
{
}

void SIL_Board::imu_read_accel(float accel[3])
{
  gazebo::math::Vector3 gravity_W_ = world_->GetPhysicsEngine()->GetGravity();
  gazebo::math::Pose T_W_I = link_->GetWorldPose();
  gazebo::math::Quaternion C_W_I = T_W_I.rot;

  #if GAZEBO_MAJOR_VERSION < 5
    gazebo::math::Vector3 velocity_current_W = link_->GetWorldLinearVel();
    // link_->GetRelativeLinearAccel() does not work sometimes with old gazebo versions.
    // This issue is solved in gazebo 5.
    gazebo::math::Vector3 acceleration = (velocity_current_W - velocity_prev_W_) / dt;
    gazebo::math::Vector3 acceleration_I = C_W_I.RotateVectorReverse(acceleration - gravity_W_);
    velocity_prev_W_ = velocity_current_W;
  #else
    gazebo::math::Vector3 acceleration_I = link_->GetRelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_);
  #endif

  accel[0] = acceleration_I.x;
  accel[1] = -acceleration_I.y;
  accel[2] = -acceleration_I.z;
}

void SIL_Board::imu_read_gyro(float gyro[3])
{
  gazebo::math::Vector3 angular_vel_I = link_->GetRelativeAngularVel();

  gyro[0] = angular_vel_I.x;
  gyro[1] = -angular_vel_I.y;
  gyro[2] = -angular_vel_I.z;
}

bool SIL_Board::imu_read_all(float accel[3], float* temperature, float gyro[3])
{
  imu_read_accel(accel);
  imu_read_gyro(gyro);
  *temperature = imu_read_temperature();
}

float SIL_Board::imu_read_temperature(void)
{
    return 27;
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
  double now = world_->GetSimTime().Double();

  // combine parts to create a measurement
  gazebo::math::Vector3 measurement = I_to_B.rot.RotateVectorReverse(inertial_magnetic_field_);

  // normalize measurement
  gazebo::math::Vector3 normalized = measurement.Normalize();

  mag[0] = normalized.x;
  mag[1] = -normalized.y;
  mag[2] = -normalized.z;
}

bool SIL_Board::mag_check(void)
{
}

bool SIL_Board::baro_present(void)
{
  return true;
}

void SIL_Board::baro_read(float *altitude, float *pressure, float *temperature)
{
    // pull z measurement out of Gazebo
    gazebo::math::Pose current_state_LFU = link_->GetWorldPose();

    // Invert measurement model for pressure and temperature
    *altitude = current_state_LFU.pos.z;
    *pressure = 101325.0*pow(1- (2.25577e-5 * current_state_LFU.pos.z), 5.25588);;
    *temperature = 27;
}

void SIL_Board::baro_calibrate()
{
}

bool SIL_Board::diff_pressure_present(void)
{
    if(mav_type_ == "fixedwing")
        return true;
    else
        return false;//this->_diff_pressure_present;
}

bool SIL_Board::diff_pressure_check(void)
{
}

void SIL_Board::diff_pressure_calibrate()
{
}

void SIL_Board::diff_pressure_set_atm(float barometric_pressure)
{
}

void SIL_Board::diff_pressure_read(float *diff_pressure, float *temperature, float *velocity)
{
  double rho_ = 1.225;
  // Calculate Airspeed
  gazebo::math::Vector3 C_linear_velocity_W_C = link_->GetRelativeLinearVel();
  double u = C_linear_velocity_W_C.x;
  double v = -C_linear_velocity_W_C.y;
  double w = -C_linear_velocity_W_C.z;

/// TODO: Wind is being applied in the inertial frame, not the body-fixed frame
//  double ur = u - wind_.N;
//  double vr = v - wind_.E;
//  double wr = w - wind_.D;
//  double Va = sqrt(pow(ur,2.0) + pow(vr,2.0) + pow(wr,2.0));
  double Va = sqrt(pow(u,2.0) + pow(v,2.0) + pow(w,2.0));

  // Invert Airpseed to get sensor measurement
  double y = rho_*Va*Va/2.0; // Page 130 in the UAV Book
//  y += pressure_bias_ + pressure_noise_sigma_*standard_normal_distribution_(random_generator_);

//  y = (y>max_pressure_)?max_pressure_:y;
//  y = (y<min_pressure_)?min_pressure_:y;

  *diff_pressure = y;
  *temperature = 27.0;
  *velocity = Va;
}

bool SIL_Board::sonar_present(void)
{
    return false;
}

bool SIL_Board::sonar_check(void)
{
}

float SIL_Board::sonar_read(void)
{
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

  //no publishers, so center everything and throttle low
  if(mav_type_ == "fixedwing" && channel == 2)
      return 1000;

  if(mav_type_ == "multirotor" && channel < 4)
      return 1000;

  return 1500;
}

void SIL_Board::pwm_write(uint8_t channel, uint16_t value)
{
    pwm_outputs_[channel] = value;
}

bool SIL_Board::pwm_lost()
{
}

// non-volatile memory
/// TODO there can only be one rosflight simulated at a time (unless they are identical)
void SIL_Board::memory_init(void)
{
}

bool SIL_Board::memory_read(void * dest, size_t len)
{
  std::ifstream memory_file;
  memory_file.open("rosflight_memory.bin", std::ios::binary);
  memory_file.read((char*) dest, len);
  memory_file.close();
  return true;
}

bool SIL_Board::memory_write(const void * src, size_t len)
{
  std::ofstream memory_file;
  memory_file.open("rosflight_memory.bin", std::ios::binary);
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
