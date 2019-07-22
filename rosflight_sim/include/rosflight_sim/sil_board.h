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

#ifndef ROSFLIGHT_SIM_SIL_BOARD_H
#define ROSFLIGHT_SIM_SIL_BOARD_H

#include <cmath>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <rosflight_msgs/RCRaw.h>

#include <rosflight_firmware/udp_board.h>

#include <rosflight_sim/gz_compat.h>

namespace rosflight_sim
{

class SIL_Board : public rosflight_firmware::UDPBoard
{
private:
  GazeboVector inertial_magnetic_field_;

  double imu_update_rate_;

  double gyro_stdev_;
  double gyro_bias_walk_stdev_;
  double gyro_bias_range_;

  double acc_stdev_;
  double acc_bias_range_;
  double acc_bias_walk_stdev_;

  double baro_bias_walk_stdev_;
  double baro_stdev_;
  double baro_bias_range_;

  double mag_bias_walk_stdev_;
  double mag_stdev_;
  double mag_bias_range_;

  double airspeed_bias_walk_stdev_;
  double airspeed_stdev_;
  double airspeed_bias_range_;

  double sonar_stdev_;
  double sonar_max_range_;
  double sonar_min_range_;

  GazeboVector gyro_bias_;
  GazeboVector acc_bias_;
  GazeboVector mag_bias_;
  double baro_bias_;
  double airspeed_bias_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
  std::uniform_real_distribution<double> uniform_distribution_;

  GazeboVector gravity_;
  double ground_altitude_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;

  ros::NodeHandle* nh_;
  ros::Subscriber rc_sub_;
  rosflight_msgs::RCRaw latestRC_;
  bool rc_received_;

  std::string mav_type_;
  int pwm_outputs_[14];  //assumes maximum of 14 channels

  // Time variables
  gazebo::common::Time boot_time_;
  uint64_t next_imu_update_time_us_;
  uint64_t imu_update_period_us_;

  void RCCallback(const rosflight_msgs::RCRaw& msg);
  bool motors_spinning();

  GazeboVector prev_vel_1_;
  GazeboVector prev_vel_2_;
  GazeboVector prev_vel_3_;
  gazebo::common::Time last_time_;

public:
  SIL_Board();

  // setup
  void init_board(void) override;
  void board_reset(bool bootloader) override;

  // clock
  uint32_t clock_millis() override;
  uint64_t clock_micros() override;
  void clock_delay(uint32_t milliseconds) override;

  // sensors
  void sensors_init() override;
  uint16_t num_sensor_errors(void) override;

  bool new_imu_data() override;
  bool imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us) override;
  void imu_not_responding_error() override;

  bool mag_present(void) override;
  void mag_read(float mag[3]) override;
  void mag_update(void) override {};

  bool baro_present(void) override;
  void baro_read(float *pressure, float *temperature) override;
  void baro_update(void) override {};

  bool diff_pressure_present(void) override;
  void diff_pressure_read(float *diff_pressure, float *temperature) override;
  void diff_pressure_update(void) override {};

  bool sonar_present(void) override;
  float sonar_read(void) override;
  void sonar_update(void) override {};

  // PWM
  // TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
  void pwm_init(uint32_t refresh_rate, uint16_t idle_pwm) override;
  void pwm_write(uint8_t channel, float value) override;
  void pwm_disable(void) override;

  //RC
  float rc_read(uint8_t channel) override;
  void rc_init(rc_type_t rc_type) override;
  bool rc_lost(void) override;


  // non-volatile memory
  void memory_init(void) override;
  bool memory_read(void * dest, size_t len) override;
  bool memory_write(const void * src, size_t len) override;

  // LEDs
  void led0_on(void) override;
  void led0_off(void) override;
  void led0_toggle(void) override;

  void led1_on(void) override;
  void led1_off(void) override;
  void led1_toggle(void) override;

  //Backup Memory
  bool has_backup_data(void) override;
  rosflight_firmware::BackupData get_backup_data(void) override;

  bool gnss_present() override;
  void gnss_update() override;

  rosflight_firmware::GNSSData gnss_read() override;
  bool gnss_has_new_data() override;
  rosflight_firmware::GNSSRaw gnss_raw_read() override;

  // Gazebo stuff
  void gazebo_setup(gazebo::physics::LinkPtr link, gazebo::physics::WorldPtr world, gazebo::physics::ModelPtr model, ros::NodeHandle* nh, std::string mav_type);
  inline const int* get_outputs() const { return pwm_outputs_; }

};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_SIL_BOARD_H
