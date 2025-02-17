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

#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "rosflight_msgs/msg/airspeed.hpp"
#include "rosflight_msgs/msg/barometer.hpp"
#include "rosflight_msgs/msg/battery_status.hpp"
#include "rosflight_msgs/msg/gnss.hpp"
#include "rosflight_msgs/msg/gnss_full.hpp"
#include "rosflight_msgs/msg/rc_raw.hpp"
#include "rosflight_sim/udp_board.hpp"

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
  long serial_delay_ns_ = 0;
  std::queue<std::tuple<long, uint8_t>> serial_delay_queue_;

  // ROS interface objects
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr imu_temperature_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_data_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Barometer>::SharedPtr baro_data_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::GNSS>::SharedPtr gnss_data_sub_;
  // rclcpp::Subscription<rosflight_msgs::msg::GNSSFull>::SharedPtr gnss_full_data_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Airspeed>::SharedPtr diff_pressure_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_data_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::BatteryStatus>::SharedPtr battery_data_sub_;

  rclcpp::Subscription<rosflight_msgs::msg::RCRaw>::SharedPtr rc_sub_;
  rosflight_msgs::msg::RCRaw latestRC_;
  bool rc_received_ = false;
  bool new_rc_data_available_;
  rclcpp::Time last_rc_message_; // TODO: This doesn't seem like it is used anywhere...

  // TODO: figure out where to define the mav_type_. Previously it was defined by Gazebo I believe
  std::string mav_type_;
  std::array<int, 14> pwm_outputs_ = {0}; // assumes maximum of 14 channels

  // Time variables
  rclcpp::Time boot_time_;

  bool new_imu_data_available_ = false;
  bool new_imu_temperature_data_available_ = false;
  bool new_mag_data_available_ = false;
  bool new_baro_data_available_ = false;
  bool new_gnss_data_available_ = false;
  bool new_gnss_full_data_available_ = false;
  bool new_diff_pressure_data_available_ = false;
  bool new_sonar_data_available_ = false;
  bool new_battery_data_available_ = false;

  bool imu_present_ = false;
  bool mag_present_ = false;
  bool baro_present_ = false;
  bool gnss_present_ = false;
  bool diff_pressure_present_ = false;
  bool sonar_present_ = false;
  bool battery_present_ = false;

  // Persistent data
  sensor_msgs::msg::Imu imu_data_;
  sensor_msgs::msg::Temperature imu_temperature_data_;
  sensor_msgs::msg::MagneticField mag_data_;
  rosflight_msgs::msg::Barometer baro_data_;
  rosflight_msgs::msg::GNSS gnss_data_;
  rosflight_msgs::msg::GNSSFull gnss_full_data_;
  rosflight_msgs::msg::Airspeed diff_pressure_data_;
  sensor_msgs::msg::Range sonar_data_;
  rosflight_msgs::msg::BatteryStatus battery_data_;

  /**
   * @brief Callback function to update RC values when new values are received.
   *
   * @param msg ROSflight RC message published on /RC topic
   */
  void RC_callback(const rosflight_msgs::msg::RCRaw & msg);
  // Subscription callbacks
  void imu_data_callback(const sensor_msgs::msg::Imu & msg);
  void imu_temperature_data_callback(const sensor_msgs::msg::Temperature & msg);
  void mag_data_callback(const sensor_msgs::msg::MagneticField & msg);
  void baro_data_callback(const rosflight_msgs::msg::Barometer & msg);
  void gnss_data_callback(const rosflight_msgs::msg::GNSS & msg);
  void diff_pressure_data_callback(const rosflight_msgs::msg::Airspeed & msg);
  void sonar_data_callback(const sensor_msgs::msg::Range & msg);
  void battery_data_callback(const rosflight_msgs::msg::BatteryStatus & msg);

  // TODO: These values don't seem to be used anywhere...
  // GazeboVector prev_vel_1_;
  // GazeboVector prev_vel_2_;
  // GazeboVector prev_vel_3_;
  // gazebo::common::Time last_time_;

  float battery_voltage_multiplier_{1.0};
  float battery_current_multiplier_{1.0};
  static constexpr size_t BACKUP_SRAM_SIZE{1024};
  uint8_t backup_memory_[BACKUP_SRAM_SIZE] = {0};

public:
  SILBoard(rclcpp::Node::SharedPtr node);

  // setup
  /**
   * @brief Sets up board with initial values and state.
   */
  void init_board() override;
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void board_reset(bool bootloader) override {};

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
  void clock_delay(uint32_t milliseconds) override {};

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
   * @brief Initializes all the sensors within the selected simulator, using provided sensor parameters.
   */
  void sensors_init() override;
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
   * @brief Initializes RC PWM values to idle values.
   *
   * @note Calls pwm_init method. Parameters are not used in SIL board.
   *
   * @param rate The refresh rate of the channel
   * @param channels The number of channels to write to.
   */
  void pwm_init_multi(const float *rate, uint32_t channels) override;
  /**
   * @brief Writes PWM value to given channel, from a value between 0 and 1. The value is scaled to
   * 1000 to 2000 and then written to the channel.
   *
   * @param channel Channel to write PWM to.
   * @param value Value between 0 and 1, to write to channel.
   */
  void pwm_write(uint8_t channel, float value) override;
  /**
   * @brief Writes PWM value to given channels, using pwm_write method. 
   *
   * @param channel Channel to write PWM to.
   * @param value Array of values between 0 and 1, to write to channel.
   */
  void pwm_write_multi(float *value, uint32_t channels) override;
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
  void rc_init(rc_type_t rc_type) override {};
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
  void memory_init() override {};
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
  void led0_on() override {};
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void led0_off() override {};
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void led0_toggle() override {};

  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void led1_on() override {};
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void led1_off() override {};
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void led1_toggle() override {};

  // Backup Memory
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void backup_memory_init() override {};
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

  inline const std::array<int, 14> & get_outputs() const { return pwm_outputs_; }

  // TODO: implement these if necessary
  bool imu_present() override;
  uint16_t sensors_errors_count() override { return 0; }
  uint16_t sensors_init_message_count() override { return 0; }
  bool sensors_init_message_good(uint16_t i) override { return true; }
  uint16_t sensors_init_message(char *message, uint16_t size, uint16_t i) { return 0; }

  void declare_parameters();
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_SIL_BOARD_H
