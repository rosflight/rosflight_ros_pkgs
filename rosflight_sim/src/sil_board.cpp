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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>

#include <rclcpp/logging.hpp>

#include "rosflight_sim/sil_board.hpp"

namespace rosflight_sim
{
SILBoard::SILBoard(rclcpp::Node::SharedPtr node)
  : UDPBoard()
  , node_(node)
{
  declare_parameters();
}

void SILBoard::declare_parameters()
{
  node_->declare_parameter("simulation_host", rclcpp::PARAMETER_STRING);
  node_->declare_parameter("simulation_port", rclcpp::PARAMETER_INTEGER);
  node_->declare_parameter("ROS_host", rclcpp::PARAMETER_STRING);
  node_->declare_parameter("ROS_port", rclcpp::PARAMETER_INTEGER);
  node_->declare_parameter("serial_delay_ns", rclcpp::PARAMETER_INTEGER);
}

void SILBoard::init_board() {
  boot_time_ = node_->get_clock()->now();

  // Set up the udp connection
  auto bind_host = node_->get_parameter_or<std::string>("simulation_host", "localhost");
  int bind_port = node_->get_parameter_or<int>("simulation_port", 14525);
  auto remote_host = node_->get_parameter_or<std::string>("ROS_host", "localhost");
  int remote_port = node_->get_parameter_or<int>("ROS_port", 14520);

  // Get communication delay parameters, in nanoseconds
  serial_delay_ns_ = node_->get_parameter_or<long>("serial_delay_ns", 0.006 * 1e9);

  set_ports(bind_host, bind_port, remote_host, remote_port);
  RCLCPP_INFO_STREAM(node_->get_logger(), "ROSflight SIL Conneced to " << remote_host
    << ":" << remote_port << " from " << bind_host << ":" << bind_port << "\n");
}

constexpr double rad2Deg(double x) { return 180.0 / M_PI * x; }
constexpr double deg2Rad(double x) { return M_PI / 180.0 * x; }

// clock
uint32_t SILBoard::clock_millis()
{
  uint32_t millis = (node_->get_clock()->now().nanoseconds() - boot_time_.nanoseconds()) * 1e-6;
  return millis;
}

uint64_t SILBoard::clock_micros()
{
  uint64_t micros = (node_->get_clock()->now().nanoseconds() - boot_time_.nanoseconds()) * 1e-3;
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
/// noise params are hard coded
void SILBoard::sensors_init()
{
  // Initialize subscribers
  imu_data_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("simulated_sensors/imu/data", 1,
      std::bind(&SILBoard::imu_data_callback, this, std::placeholders::_1));

  imu_temperature_data_sub_ = node_->create_subscription<sensor_msgs::msg::Temperature>("simulated_sensors/imu/temperature", 1,
      std::bind(&SILBoard::imu_temperature_data_callback, this, std::placeholders::_1));

  mag_data_sub_ = node_->create_subscription<sensor_msgs::msg::MagneticField>("simulated_sensors/mag", 1,
      std::bind(&SILBoard::mag_data_callback, this, std::placeholders::_1));

  baro_data_sub_ = node_->create_subscription<rosflight_msgs::msg::Barometer>("simulated_sensors/baro", 1,
      std::bind(&SILBoard::baro_data_callback, this, std::placeholders::_1));

  gnss_data_sub_ = node_->create_subscription<rosflight_msgs::msg::GNSS>("simulated_sensors/gnss", 1,
      std::bind(&SILBoard::gnss_data_callback, this, std::placeholders::_1));

  diff_pressure_data_sub_ = node_->create_subscription<rosflight_msgs::msg::Airspeed>("simulated_sensors/diff_pressure", 1,
      std::bind(&SILBoard::diff_pressure_data_callback, this, std::placeholders::_1));

  sonar_data_sub_ = node_->create_subscription<sensor_msgs::msg::Range>("simulated_sensors/sonar", 1,
      std::bind(&SILBoard::sonar_data_callback, this, std::placeholders::_1));

  battery_data_sub_ = node_->create_subscription<rosflight_msgs::msg::BatteryStatus>("simulated_sensors/battery", 1,
      std::bind(&SILBoard::battery_data_callback, this, std::placeholders::_1));
}

void SILBoard::imu_data_callback(const sensor_msgs::msg::Imu & msg)
{
  imu_data_ = msg;
}

void SILBoard::imu_temperature_data_callback(const sensor_msgs::msg::Temperature & msg)
{
  imu_temperature_data_ = msg;
}

void SILBoard::mag_data_callback(const sensor_msgs::msg::MagneticField & msg)
{
  mag_data_ = msg;
}

void SILBoard::baro_data_callback(const rosflight_msgs::msg::Barometer & msg)
{
  baro_data_ = msg;
}

void SILBoard::gnss_data_callback(const rosflight_msgs::msg::GNSS & msg)
{
  gnss_data_ = msg;
}

void SILBoard::diff_pressure_data_callback(const rosflight_msgs::msg::Airspeed & msg)
{
  diff_pressure_data_ = msg;
}

void SILBoard::sonar_data_callback(const sensor_msgs::msg::Range & msg)
{
  sonar_data_ = msg;
}

void SILBoard::battery_data_callback(const rosflight_msgs::msg::BatteryStatus & msg)
{
  battery_data_ = msg;
}

bool SILBoard::imu_read(rosflight_firmware::ImuStruct * imu)
{
  // Populate the data from the last imu data received by the sensor
  imu->accel[0] = imu_data_.linear_acceleration.x;
  imu->accel[1] = imu_data_.linear_acceleration.y;
  imu->accel[2] = imu_data_.linear_acceleration.z;

  imu->temperature = imu_temperature_data_.temperature + 273.15;  // Convert to degrees Kelvin from Celcius

  imu->gyro[0] = imu_data_.angular_velocity.x;
  imu->gyro[1] = imu_data_.angular_velocity.y;
  imu->gyro[2] = imu_data_.angular_velocity.z;

  // TODO: is this right?
  // Time the data was read
  imu->header.timestamp = imu_data_.header.stamp.sec * 1e6
    + imu_data_.header.stamp.nanosec * 1e-3;

  return true;
}

bool SILBoard::mag_read(rosflight_firmware::MagStruct * mag)
{
  // TODO: should this be in tesla or nanotesla?
  mag->flux[0] = mag_data_.magnetic_field.x;
  mag->flux[1] = mag_data_.magnetic_field.y;
  mag->flux[2] = mag_data_.magnetic_field.z;

  mag->header.timestamp = mag_data_.header.stamp.sec * 1e6
    + mag_data_.header.stamp.nanosec * 1e-3;
  return true;
}

bool SILBoard::baro_read(rosflight_firmware::PressureStruct * baro)
{
  baro->pressure = baro_data_.pressure;
  baro->temperature = baro_data_.temperature;
  
  baro->header.timestamp = baro_data_.header.stamp.sec * 1e6
    + baro_data_.header.stamp.nanosec * 1e-3;
  return true;
}

bool SILBoard::diff_pressure_read(rosflight_firmware::PressureStruct * diff_pressure)
{
  diff_pressure->pressure = diff_pressure_data_.differential_pressure;
  diff_pressure->temperature = diff_pressure_data_.temperature;
  diff_pressure->header.timestamp = diff_pressure_data_.header.stamp.sec * 1e6
    + diff_pressure_data_.header.stamp.nanosec * 1e-3;
  return true;
}

bool SILBoard::sonar_read(rosflight_firmware::RangeStruct * sonar)
{
  sonar->range = sonar_data_.range;
  return true;
}

bool SILBoard::battery_read(rosflight_firmware::BatteryStruct * batt)
{
  batt->voltage = battery_data_.voltage * battery_voltage_multiplier_;
  batt->current = battery_data_.current * battery_current_multiplier_;
  batt->header.timestamp = battery_data_.header.stamp.sec * 1e6
    + battery_data_.header.stamp.nanosec * 1e-3;
  return true;
}

bool SILBoard::gnss_read(rosflight_firmware::GNSSData * gnss)
{
  gnss->unix_seconds = gnss_data_.header.stamp.sec;
  gnss->nano = gnss_data_.header.stamp.nanosec;
  gnss->time_of_week = gnss->unix_seconds * 1e6
    + gnss->nano * 1e-3;  // Only used internal to the firmware. Pseudo GNSS iTOW in ms

  // Cast to rosflight_firmware::GNSSFixType first for error checking from enum class
  gnss->fix_type = static_cast<uint8_t>(
    static_cast<rosflight_firmware::GNSSFixType>(gnss_data_.fix_type));

  gnss->num_sat = gnss_data_.num_sat;
  gnss->lat = (int32_t) gnss_data_.lat * 1e7; // Convert DDS into 100's of nanodegrees
  gnss->lon = (int32_t) gnss_data_.lon * 1e7; // Convert DDS into 100's of nanodegrees
  gnss->height_msl = (int32_t) gnss_data_.alt * 1e3; // m to mm
  gnss->h_acc = (uint32_t) gnss_data_.horizontal_accuracy * 1e3; // m to mm
  gnss->v_acc = (uint32_t) gnss_data_.vertical_accuracy * 1e3; // m to mm

  gnss->vel_n = (int32_t) gnss_data_.vel_n * 1e3; // m to mm
  gnss->vel_e = (int32_t) gnss_data_.vel_e * 1e3; // m to mm
  gnss->vel_d = (int32_t) gnss_data_.vel_d * 1e3; // m to mm
  gnss->speed_accy = (uint32_t) gnss_data_.speed_accuracy * 1e3; // m to mm

  gnss->header.timestamp = gnss_data_.header.stamp.sec * 1e6
    + gnss_data_.header.stamp.nanosec * 1e-3;

  return true;
}

void SILBoard::battery_voltage_set_multiplier(double multiplier)
{
  battery_voltage_multiplier_ = (float) multiplier;
}

void SILBoard::battery_current_set_multiplier(double multiplier)
{
  battery_current_multiplier_ = (float) multiplier;
}

// PWM
void SILBoard::pwm_init(const float * rate, uint32_t channels)
{
  rc_received_ = false;
  // TODO: Switch channel assignments to be a ROS parameter
  latestRC_.values[0] = 1500; // x
  latestRC_.values[1] = 1500; // y
  latestRC_.values[3] = 1500; // z
  latestRC_.values[2] = 1000; // F
  latestRC_.values[4] = 1000; // attitude override
  latestRC_.values[5] = 1000; // arm

  for (auto & pwm_output : pwm_outputs_) {
    pwm_output = 1000;
  }

  rc_sub_ = node_->create_subscription<rosflight_msgs::msg::RCRaw>(
    "sim/RC", 1, std::bind(&SILBoard::RC_callback, this, std::placeholders::_1));
}

bool SILBoard::rc_read(rosflight_firmware::RcStruct * rc_struct)
{
  if (rc_sub_->get_publisher_count() > 0) {
    for (uint16_t i=0; i < 8; ++i) {
      rc_struct->chan[i] = static_cast<float>(latestRC_.values[i] - 1000) / 1000.0f;
    }

    return true;
  }

  // no publishers, set throttle low and center everything else
  for (uint16_t i=0; i < 8; ++i) {
    // TODO: Change this channel to be a parameter
    if (i == 2) {
      rc_struct->chan[i] = 0.0;
    } else {
      rc_struct->chan[i] = 0.5;
    }
  }

  return false;
}

void SILBoard::pwm_write(float * value, uint32_t channels)
{
  for (uint32_t i=0; i<channels; ++i) {
    float val = value[i];
    val = (val < 0) ? 0 : val;
    val = (val > 1) ? 1 : val;

    pwm_outputs_[i] = 1000 + (uint16_t) (1000 * val);
  }
}

void SILBoard::pwm_disable()
{
  float vals[14] = {0};
  pwm_write(vals, 14);
}

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
  new_rc_data_available_ = true;
  last_rc_message_ = node_->get_clock()->now();
  latestRC_ = msg;
}

} // namespace rosflight_sim
