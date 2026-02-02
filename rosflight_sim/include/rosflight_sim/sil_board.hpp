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
  rclcpp::Subscription<rosflight_msgs::msg::Airspeed>::SharedPtr diff_pressure_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_data_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::BatteryStatus>::SharedPtr battery_data_sub_;

  rclcpp::Subscription<rosflight_msgs::msg::RCRaw>::SharedPtr rc_sub_;
  rosflight_msgs::msg::RCRaw latestRC_;
  bool rc_received_ = false;
  bool new_rc_data_available_;
  rclcpp::Time
    last_rc_message_; // TODO: This isn't actually used anywhere... Do we need to handle rc_message time out in SIL board?

  // TODO: figure out where to define the mav_type_. Previously it was defined by Gazebo in a xacro file, I think. Do we need it anymore?
  std::string mav_type_;
  std::array<uint16_t, 14> pwm_outputs_ = {0}; // assumes maximum of 14 channels

  // Time variables
  rclcpp::Time boot_time_;

  bool imu_has_new_data_available_ = false;
  bool baro_has_new_data_available_ = false;
  bool mag_has_new_data_available_ = false;
  bool diff_pressure_has_new_data_available_ = false;
  bool gnss_has_new_data_available_ = false;
  bool range_has_new_data_available_ = false;
  bool battery_has_new_data_available_ = false;

  // Persistent data
  sensor_msgs::msg::Imu imu_data_;
  sensor_msgs::msg::Temperature imu_temperature_data_;
  sensor_msgs::msg::MagneticField mag_data_;
  rosflight_msgs::msg::Barometer baro_data_;
  rosflight_msgs::msg::GNSS gnss_data_;
  rosflight_msgs::msg::Airspeed diff_pressure_data_;
  sensor_msgs::msg::Range range_data_;
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
  void range_data_callback(const sensor_msgs::msg::Range & msg);
  void battery_data_callback(const rosflight_msgs::msg::BatteryStatus & msg);

  float battery_voltage_multiplier_;
  float battery_current_multiplier_;
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
   * @brief Gets current FCU time in milliseconds based on node time.
   *
   * @return Current FCU time in milliseconds.
   */
  uint32_t clock_millis() override;
  /**
   * @brief Gets current FCU time in microseconds based on node time.
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
   * @brief Returns simulated IMU data from sensors node
   *
   * @param imu rosflight_firmware imu data structure to fill
   */
  bool imu_read(rosflight_firmware::ImuStruct * imu) override;

  /**
   * @brief Returns magnetometer data generated from sensors node
   *
   * @param mag rosflight_firmware mag data structure to fill
   * @return true if successful.
   */
  bool mag_read(rosflight_firmware::MagStruct * mag) override;

  /**
   * @brief Returns baro data generated from sensors node
   *
   * @param baro rosflight_firmware baro data structure to fill
   * @return true if successful.
   */
  bool baro_read(rosflight_firmware::PressureStruct * baro) override;

  /**
   * @brief Returns diff_pressure data generated from sensors node
   *
   * @param diff_pressure rosflight_firmware diff_pressure data structure to fill
   * @return true if successful.
   */
  bool diff_pressure_read(rosflight_firmware::PressureStruct * diff_pressure) override;

  /**
   * @brief Returns range data generated from sensors node
   *
   * @param range rosflight_firmware range data structure to fill
   * @return true if successful.
   */
  bool range_read(rosflight_firmware::RangeStruct * range) override;

  // PWM
  // TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
  /**
   * @brief Initializes RC PWM values to idle values.
   *
   * @note Does not use function arguments. Refresh rate is determined by RC publish rate, idle
   * value is hard coded.
   *
   * @param rate Does nothing. Follows /RC topic's publish rate.
   * @param channels Does nothing. Hardcoded
   */
  void pwm_init(const float * rate, uint32_t channels) override;
  /**
   * @brief Writes PWM value to given channel, from a value between 0 and 1. The value is scaled to
   * 1000 to 2000 and then written to the channel.
   *
   * @param value Value between 0 and 1, to write to channel.
   * @param channels number of PWM channels
   */
  void pwm_write(float * value, uint32_t channels) override;
  /**
   * @brief Sets all PWM values to 1000.
   */
  void pwm_disable() override;

  // RC
  /**
   * @brief Gets latest RC values published on /RC. If nothing in publishing on /RC, values are set
   * to idle.
   *
   * @param range rosflight_firmware range data structure to fill
   * @return true if successful.
   */
  bool rc_read(rosflight_firmware::RcStruct * rc_struct) override;
  /**
   * @brief Function required to be overridden, but not used by sim.
   */
  void rc_init(rc_type_t rc_type) override {};

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
   * @brief Generates GNSS data based on truth and noise/bias parameters.
   *
   * @param gnss GnssStruct object to update.
   *
   * @return true if successful.
   */
  bool gnss_read(rosflight_firmware::GnssStruct * gnss) override;
  /**
   * @brief Creates battery data based on sim model.
   *
   * @param batt rosflight_firmware batt struct
   *
   * @return true if successful.
   */
  bool battery_read(rosflight_firmware::BatteryStruct * batt) override;
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

  inline const std::array<uint16_t, 14> & get_outputs() const { return pwm_outputs_; }

  // Unused virtual functions
  uint16_t sensors_errors_count() override { return 0; }
  uint16_t sensors_init_message_count() override { return 0; }
  bool sensors_init_message_good(uint16_t i) override { return true; }
  uint16_t sensors_init_message(char * message, uint16_t size, uint16_t i) { return 0; }

  void declare_parameters();
};

} // namespace rosflight_sim

#endif // ROSFLIGHT_SIM_SIL_BOARD_H
