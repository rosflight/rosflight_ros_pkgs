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
#include <cstdbool>
#include <cstddef>
#include <cstdint>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>

#include <udp_board.h>

#include <rosflight_sim/gz_compat.hpp>

namespace rosflight_sim
{
class SIL_Board : public rosflight_firmware::UDPBoard
{
private:
  GazeboVector inertial_magnetic_field_;

  double imu_update_rate_ = 0;

  long serial_delay_ns_ = 0;
  std::queue<std::tuple<long, uint8_t>> serial_delay_queue_;

  double gyro_stdev_ = 0;
  double gyro_bias_walk_stdev_ = 0;
  double gyro_bias_range_ = 0;

  double acc_stdev_ = 0;
  double acc_bias_range_ = 0;
  double acc_bias_walk_stdev_ = 0;

  double baro_bias_walk_stdev_ = 0;
  double baro_stdev_ = 0;
  double baro_bias_range_ = 0;

  double mag_bias_walk_stdev_ = 0;
  double mag_stdev_ = 0;
  double mag_bias_range_ = 0;

  double airspeed_bias_walk_stdev_ = 0;
  double airspeed_stdev_ = 0;
  double airspeed_bias_range_ = 0;

  double sonar_stdev_ = 0;
  double sonar_max_range_ = 0;
  double sonar_min_range_ = 0;

  double horizontal_gps_stdev_ = 0;
  double vertical_gps_stdev_ = 0;
  double gps_velocity_stdev_ = 0;

  GazeboVector gyro_bias_;
  GazeboVector acc_bias_;
  GazeboVector mag_bias_;
  double baro_bias_ = 0;
  double airspeed_bias_ = 0;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
  std::uniform_real_distribution<double> uniform_distribution_;

  GazeboVector gravity_;
  double origin_latitude_ = 0;
  double origin_longitude_ = 0;
  double origin_altitude_ = 0;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<rosflight_msgs::msg::RCRaw>::SharedPtr rc_sub_;
  rosflight_msgs::msg::RCRaw latestRC_;
  bool rc_received_ = false;
  rclcpp::Time last_rc_message_;

  std::string mav_type_;
  int pwm_outputs_[14] = {0}; // assumes maximum of 14 channels

  // Time variables
  gazebo::common::Time boot_time_;
  uint64_t next_imu_update_time_us_ = 0;
  uint64_t imu_update_period_us_ = 0;

  void RCCallback(const rosflight_msgs::msg::RCRaw &msg);
  bool motors_spinning();

  GazeboVector prev_vel_1_;
  GazeboVector prev_vel_2_;
  GazeboVector prev_vel_3_;
  gazebo::common::Time last_time_;

  float battery_voltage_multiplier{1.0};
  float battery_current_multiplier{1.0};
  static constexpr size_t BACKUP_SRAM_SIZE{1024};
  uint8_t backup_memory_[BACKUP_SRAM_SIZE] = {0};

public:
  SIL_Board();

  // setup
  void init_board() override;
  void board_reset(bool bootloader) override;

  // clock
  uint32_t clock_millis() override;
  uint64_t clock_micros() override;
  void clock_delay(uint32_t milliseconds) override;

  // serial
  uint8_t serial_read() override;
  uint16_t serial_bytes_available() override;

  // sensors
  void sensors_init() override;
  uint16_t num_sensor_errors() override;

  bool new_imu_data() override;
  bool imu_read(float accel[3], float * temperature, float gyro[3], uint64_t * time_us) override;
  void imu_not_responding_error() override;

  bool mag_present() override;
  void mag_read(float mag[3]) override;
  void mag_update() override {};

  bool baro_present() override;
  void baro_read(float * pressure, float * temperature) override;
  void baro_update() override {};

  bool diff_pressure_present() override;
  void diff_pressure_read(float * diff_pressure, float * temperature) override;
  void diff_pressure_update() override {};

  bool sonar_present() override;
  float sonar_read() override;
  void sonar_update() override {};

  // PWM
  // TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
  void pwm_init(uint32_t refresh_rate, uint16_t idle_pwm) override;
  void pwm_write(uint8_t channel, float value) override;
  void pwm_disable() override;

  // RC
  float rc_read(uint8_t channel) override;
  void rc_init(rc_type_t rc_type) override;
  bool rc_lost() override;

  // non-volatile memory
  void memory_init() override;
  bool memory_read(void * dest, size_t len) override;
  bool memory_write(const void * src, size_t len) override;

  // LEDs
  void led0_on() override;
  void led0_off() override;
  void led0_toggle() override;

  void led1_on() override;
  void led1_off() override;
  void led1_toggle() override;

  // Backup Memory
  void backup_memory_init() override;
  bool backup_memory_read(void * dest, size_t len) override;
  void backup_memory_write(const void * src, size_t len) override;
  void backup_memory_clear(size_t len) override;

  bool gnss_present() override;
  void gnss_update() override;

  rosflight_firmware::GNSSData gnss_read() override;
  bool gnss_has_new_data() override;
  rosflight_firmware::GNSSFull gnss_full_read() override;

  bool battery_voltage_present() const override;
  float battery_voltage_read() const override;
  void battery_voltage_set_multiplier(double multiplier) override;

  bool battery_current_present() const override;
  float battery_current_read() const override;
  void battery_current_set_multiplier(double multiplier) override;

  // Gazebo stuff
  void gazebo_setup(gazebo::physics::LinkPtr link,
                    gazebo::physics::WorldPtr world,
                    gazebo::physics::ModelPtr model,
                    rclcpp::Node::SharedPtr node,
                    std::string mav_type);
  inline const int * get_outputs() const { return pwm_outputs_; }
#if GAZEBO_MAJOR_VERSION >= 9
  gazebo::common::SphericalCoordinates sph_coord_;
#endif
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_SIL_BOARD_H
