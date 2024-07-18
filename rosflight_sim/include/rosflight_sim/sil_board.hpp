/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
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

#include "sensors.h"
#include <cmath>
#include <cstdbool>
#include <cstddef>
#include <cstdint>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/detail/gnss_full__struct.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>

#include <rosflight_sim/gz_compat.hpp>
#include <rosflight_sim/udp_board.hpp>

namespace rosflight_sim
{
/**
 * @brief ROSflight firmware board implementation for simulator. This class handles sensors,
 * actuators, and FCU clock and memory for the firmware. It also adds a simulated serial delay. It
 * inherits from UDP board, which establishes a communication link over UDP.
 */
class SILBoard : public UDPBoard
{
private:
  GazeboVector inertial_magnetic_field_;

  double imu_update_rate_ = 0;
  double mag_update_rate_ = 0;
  double gnss_update_rate_ = 0;
  double baro_update_rate_ = 0;
  double diff_pressure_update_rate_ = 0;
  double sonar_update_rate_ = 0;
  double rc_update_rate_ = 0;
  double battery_update_rate_ = 0;

  long serial_delay_ns_ = 0;
  std::queue<std::tuple<long, uint8_t>> serial_delay_queue_;

  double gyro_stdev_ = 0;
  double gyro_bias_walk_stdev_ = 0;
  double gyro_bias_range_ = 0;

  double acc_stdev_ = 0;
  double acc_bias_range_ = 0;
  double acc_bias_walk_stdev_ = 0;

  double mass_ = 0; // Use actual values since these are divisors.
  double rho_ = 0;  // This will prevent a division by zero.

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

  double f_x = 0;
  double f_y = 0;
  double f_z = 0;

  GazeboVector gyro_bias_;
  GazeboVector acc_bias_;
  GazeboVector mag_bias_;
  double baro_bias_ = 0;
  double airspeed_bias_ = 0;

  std::default_random_engine bias_generator_;
  std::default_random_engine noise_generator_;
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
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr forces_sub_;
  rosflight_msgs::msg::RCRaw latestRC_;
  bool rc_received_ = false;
  rclcpp::Time last_rc_message_;

  std::string mav_type_;
  int pwm_outputs_[14] = {0}; // assumes maximum of 14 channels

  // Time variables
  gazebo::common::Time boot_time_;
  uint64_t next_imu_update_time_us_ = 0;
  uint64_t next_mag_update_time_us_ = 0;
  uint64_t next_gnss_update_time_us_ = 0;
  uint64_t next_baro_update_time_us_ = 0;
  uint64_t next_diff_pressure_update_time_us_ = 0;
  uint64_t next_sonar_update_time_us_ = 0;
  uint64_t next_rc_update_time_us_ = 0;
  uint64_t next_battery_update_time_us_ = 0;
  uint64_t imu_update_period_us_ = 0;
  uint64_t mag_update_period_us_ = 0;
  uint64_t gnss_update_period_us_ = 0;
  uint64_t baro_update_period_us_ = 0;
  uint64_t diff_pressure_update_period_us_ = 0;
  uint64_t sonar_update_period_us_ = 0;
  uint64_t rc_update_period_us_ = 0;
  uint64_t battery_update_period_us_ = 0;

  /**
   * @brief Callback function to update RC values when new values are received.
   *
   * @param msg ROSflight RC message published on /RC topic
   */
  void RC_callback(const rosflight_msgs::msg::RCRaw & msg);
  /**
   * @brief Callback function to update the aerodynamic forces.
   *
   * @param msg Geometry message that contains the forces and moments.
   */
  void forces_callback(const geometry_msgs::msg::TwistStamped & msg);
  /**
   * @brief Checks the current pwm value for throttle to see if the motor pwm is above minimum, and that
   * the motors should be spinning.
   *
   * @return true if throttle pwm is greater than 1100, false if less than or equal to.
   */
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
  SILBoard();

  // setup
  /**
   * @brief Sets up board with initial values and state.
   */
  void init_board() override;
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void board_reset(bool bootloader) override{};

  // clock
  /**
   * @brief Gets current FCU time in milliseconds based on Gazebo time.
   *
   * @return Current FCU time in milliseconds.
   */
  uint32_t clock_millis() override;
  /**
   * @brief Gets current FCU time in microseconds based on Gazebo time.
   *
   * @return Current FCU time in microseconds
   */
  uint64_t clock_micros() override;
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void clock_delay(uint32_t milliseconds) override{};

  // serial
  /**
   * @brief Function that is called in firmware loop to read from serial buffer. Overriden to
   * implement serial delay for simulation purposes.
   */
  uint8_t serial_read() override;
  /**
   * @brief Function to check if bytes are in the serial communication buffer. Overriden to
   * implement a serial delay for simulation purposes.
   *
   * @return true if bytes are ready.
   */
  uint16_t serial_bytes_available() override;

  // sensors
  /**
   * @brief Initializes all the sensors within Gazebo, using provided sensor parameters.
   */
  void sensors_init() override;
  /**
   * @brief Function used to return number of unhealthy sensors. Currently returns 0.
   *
   * @return 0
   */
  uint16_t num_sensor_errors() override;

  /**
   * @brief Checks if there is new IMU data ready to be processed.
   *
   * @return true if unprocessed IMU data exists.
   */
  bool imu_has_new_data() override;
  /**
   * @brief Generates simulated IMU data from truth data from Gazebo. Utilizes noise, bias, and walk
   * parameters provided. All data is returned through the values given as the function arguments.
   *
   * @param accel Acceleration values to populate.
   * @param temperature IMU temperature value to populate.
   * @param gyro Gyro values to populate.
   * @param time_us Time value to populate.
   */
  bool imu_read(float accel[3], float * temperature, float gyro[3], uint64_t * time_us) override;
  /**
   * @brief Prints a ROS error stating that the IMU is not responding.
   */
  void imu_not_responding_error() override;

  /**
   * @brief Function used to check if a magnetometer is present. Currently always returns true.
   *
   * @return true
   */
  bool mag_present() override;
  /**
   * @brief Generates magnetometer data based on Gazebo orientation and given noise/bias parameters.
   *
   * @param mag Magnetometer values to populate.
   * @return true if successful.
   */
  bool mag_read(float mag[3]) override;
  /**
   * @brief Checks to see if it has been enough time to warrant new data.
   * @return true if mag has new data.
   */
  bool mag_has_new_data() override;

  /**
   * @brief Function used to check if a barometer is present. Currently returns true.
   *
   * @return true
   */
  bool baro_present() override;
  /**
   * @brief Generates barometer measurement based on Gazebo altitude and noise/bias parameters.
   *
   * @param pressure Pressure value to populate.
   * @param temperature Temperature value to populate.
   *
   * @return true if successful.
   */
  bool baro_read(float * pressure, float * temperature) override;
  /**
   * @brief Checks to see if it has been enough time to warrant new data.
   * @return true if baro has new data.
   */
  bool baro_has_new_data() override;

  /**
   * @brief Checks if a pitot tube sensor is present. Returns true if sim is a fixedwing sim.
   *
   * @return true if fixedwing sim, false otherwise.
   */
  bool diff_pressure_present() override;
  /**
   * @brief Generates a differential pressure measurement based on Gazebo speed and noise/bias
   * parameters.
   *
   * @param diff_pressure Differential pressure value to populate.
   * @param temperature Temperature value to populate.
   *
   * @return true if successful.
   */
  bool diff_pressure_read(float * diff_pressure, float * temperature) override;
  /**
   * @brief Checks to see if it has been enough time to warrant new data.
   *
   * @return true if diff_pressure sensor has new data.
   */
  bool diff_pressure_has_new_data() override;

  /**
   * @brief Function used to see if a sonar altitude sensor is present. Currently returns true.
   *
   * @return true
   */
  bool sonar_present() override;
  /**
   * @brief Generates sonar reading based on min/max range and noise parameters. Based on Gazebo
   * altitude value.
   *
   * @param range measurement to update.
   *
   * @note Currently does not take UAV attitude into account, only absolute altitude.
   *
   * @return true if successful.
   */
  bool sonar_read(float * range) override;
  /**
   * @brief Checks to see if it has been enough time to warrant new data.
   *
   * @return true if sonar sensor has new data.
   */
  bool sonar_has_new_data() override;

  // PWM
  // TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
  /**
   * @brief Initializes RC PWM values to idle values.
   *
   * @note Does not use function arguments. Refresh rate is determined by RC publish rate, idle
   * value is hard coded.
   *
   * @param refresh_rate Does nothing. Follows /RC topic's publish rate.
   * @param idle_pwm Does nothing. Hardcoded.
   */
  void pwm_init(uint32_t refresh_rate, uint16_t idle_pwm) override;
  /**
   * @brief Writes PWM value to given channel, from a value between 0 and 1. The value is scaled to
   * 1000 to 2000 and then written to the channel.
   *
   * @param channel Channel to write PWM to.
   * @param value Value between 0 and 1, to write to channel.
   */
  void pwm_write(uint8_t channel, float value) override;
  /**
   * @brief Sets all PWM values to 1000.
   */
  void pwm_disable() override;

  // RC
  /**
   * @brief Gets latest RC values published on /RC. If nothing in publishing on /RC, values are set
   * to idle.
   *
   * @param channel Channel to read value from.
   * @return 0.5
   */
  float rc_read(uint8_t chan) override;
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void rc_init(rc_type_t rc_type) override{};
  /**
   * @brief Function used to check if RC connection is present. Currently returns false if anything
   * is ever published on RC.
   *
   * @return False if /RC has had a message published, true otherwise.
   */
  bool rc_lost() override;
  /**
   * @brief Checks to see if it has been enough time to warrant new data.
   *
   * @return true if rc has new data.
   */
  bool rc_has_new_data() override;

  // non-volatile memory
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void memory_init() override{};
  /**
   * @brief Reads data from memory file.
   *
   * @param dest Memory location to read from (?)
   * @param len Length of memory to read (?)
   * @return True if read successfully, false if otherwise.
   */
  bool memory_read(void * dest, size_t len) override;
  /**
   * @brief Writes data to memory file.
   *
   * @param dest Memory location to write to (?)
   * @param len Length of memory to write to (?)
   * @return True if read successfully, false if otherwise.
   */
  bool memory_write(const void * src, size_t len) override;

  // LEDs
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void led0_on() override{};
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void led0_off() override{};
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void led0_toggle() override{};

  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void led1_on() override{};
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void led1_off() override{};
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void led1_toggle() override{};

  // Backup Memory
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void backup_memory_init() override{};
  /**
   * @brief Reads data from backup memory object.
   *
   * @param dest Memory location to read from (?)
   * @param len Length of memory to read (?)
   * @return True if read successfully, false if otherwise.
   */
  bool backup_memory_read(void * dest, size_t len) override;
  /**
   * @brief Writes data to backup memory object.
   *
   * @param dest Memory location to write to (?)
   * @param len Length of memory to write to (?)
   * @return True if read successfully, false if otherwise.
   */
  void backup_memory_write(const void * src, size_t len) override;
  /**
   * @brief Clears data from the backup memory, starting at the beginning and equal to the length
   * of the input argument provided.
   *
   * @param len Length of data to clear.
   */
  void backup_memory_clear(size_t len) override;
  /**
   * @brief Function used to check if GNSS is present. Currenlty returns true.
   *
   * @return true
   */
  bool gnss_present() override;
  /**
   * @brief Generates GNSS data based on Gazebo truth and noise/bias parameters.
   *
   * @param gnss GNSSData object to update.
   * @param gnss_full GNSSFull object to update.
   *
   * @return true if successful.
   */
  bool gnss_read(rosflight_firmware::GNSSData * gnss,
                 rosflight_firmware::GNSSFull * gnss_full) override;
  /**
   * @brief Checks to see if it has been enough time to warrant new data.
   *
   * @return true
   */
  bool gnss_has_new_data() override;
  /**
   * @brief Function used to check if a battery is present. Currenlty returns true.
   *
   * @return true
   */
  bool battery_present() override;
  /**
   * @brief Checks to see if it has been enough time to warrant new data.
   *
   * @return true if battery has new data.
   */
  bool battery_has_new_data() override;
  /**
   * @brief Creates battery data based on sim model.
   *
   * @param voltage The voltage float to update
   * @param current The current float to update
   *
   * @return true if successful.
   */
  bool battery_read(float * voltage, float * current) override;
  /**
   * @brief Sets battery voltage calibration constant.
   *
   * @param multiplier Voltage calibration constant
   */
  void battery_voltage_set_multiplier(double multiplier) override;
  /**
   * @brief Sets battery current calibration constant.
   *
   * @param multiplier Current calibration constant
   */
  void battery_current_set_multiplier(double multiplier) override;

  // Gazebo stuff
  /**
   * @brief Initializes the firmware with ROS parameters and Gazebo data types.
   *
   * @param link Gazebo link pointer, provided by ModelPlugin::Load function
   * @param world Gazebo world pointer, provided by ModelPlugin::Load function
   * @param model Gazebo model pointer, provided by ModelPlugin::Load function
   * @param node ROS node pointer, provided by ModelPlugin::Load function
   * @param mav_type Simulation type
   */
  void gazebo_setup(gazebo::physics::LinkPtr link, gazebo::physics::WorldPtr world,
                    gazebo::physics::ModelPtr model, rclcpp::Node::SharedPtr node,
                    std::string mav_type);
  inline const int * get_outputs() const { return pwm_outputs_; }
  gazebo::common::SphericalCoordinates sph_coord_;
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_SIL_BOARD_H
