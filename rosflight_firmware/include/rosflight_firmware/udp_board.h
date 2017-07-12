/*
 * Copyright (c) 2017 Daniel Koch, BYU MAGICC Lab.
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

/**
 * \file udp_board.h
 * \author Daniel Koch
 */

#ifndef ROSFLIGHT_FIRMWARE_UDP_BOARD_H
#define ROSFLIGHT_FIRMWARE_UDP_BOARD_H

#include <list>
#include <mutex>
#include <string>
#include <thread>

#include <boost/asio.hpp>

#include "board.h"
#include "mavlink.h"

namespace rosflight_firmware
{

class UDPBoard : public Board
{
public:
  UDPBoard(std::string bind_host = DEFAULT_BIND_HOST,
           uint16_t bind_port = DEFAULT_BIND_PORT,
           std::string remote_host = DEFAULT_REMOTE_HOST,
           uint16_t remote_port = DEFAULT_REMOTE_PORT);
  ~UDPBoard();

  // function overrides (serial)
  void serial_init(uint32_t baud_rate) override;
  void serial_write(uint8_t byte) override;
  uint16_t serial_bytes_available(void) override;
  uint8_t serial_read(void) override;

  // setup
  virtual void init_board(void) = 0;
  virtual void board_reset(bool bootloader) = 0;

  // clock
  virtual uint32_t clock_millis() = 0;
  virtual uint64_t clock_micros() = 0;
  virtual void clock_delay(uint32_t milliseconds) = 0;

  // sensors
  virtual void sensors_init() = 0;
  virtual uint16_t num_sensor_errors(void)  = 0;

  virtual bool new_imu_data() = 0;
  virtual void imu_read_accel(float accel[3]) = 0;
  virtual void imu_read_gyro(float gyro[3]) = 0;
  virtual bool imu_read_all(float accel[3], float *temperature, float gyro[3], uint64_t* time) = 0;
  virtual float imu_read_temperature(void) = 0;
  virtual void imu_not_responding_error(void) = 0;

  virtual bool mag_check(void) = 0;
  virtual bool mag_present(void) = 0;
  virtual void mag_read(float mag[3]) = 0;

  virtual bool baro_present(void) = 0;
  virtual bool baro_check(void) = 0;
  virtual void baro_read(float *altitude, float *pressure,
                         float *temperature) = 0; // TODO move altitude calculation outside this function
  virtual void baro_calibrate() = 0;

  virtual bool diff_pressure_present(void) = 0;
  virtual bool diff_pressure_check(void) = 0;
  virtual void diff_pressure_set_atm(float barometric_pressure) = 0;
  virtual void diff_pressure_calibrate() = 0;
  virtual void diff_pressure_read(float *diff_pressure, float *temperature,
                                  float *velocity) = 0; // TODO move velocity calculation outside this function

  virtual bool sonar_present(void) = 0;
  virtual bool sonar_check(void) = 0;
  virtual float sonar_read(void) = 0;

  // PWM
  virtual void pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm) = 0;
  virtual bool pwm_lost() = 0;
  virtual uint16_t pwm_read(uint8_t channel) = 0;
  virtual void pwm_write(uint8_t channel, uint16_t value) = 0;

  // non-volatile memory
  virtual void memory_init(void) = 0;
  virtual bool memory_read(void *dest, size_t len) = 0;
  virtual bool memory_write(const void *src, size_t len) = 0;

  // LEDs
  virtual void led0_on(void) = 0;
  virtual void led0_off(void) = 0;
  virtual void led0_toggle(void) = 0;

  virtual void led1_on(void) = 0;
  virtual void led1_off(void) = 0;
  virtual void led1_toggle(void) = 0;

private:
  static constexpr char DEFAULT_BIND_HOST[] = "localhost";
  static constexpr uint16_t DEFAULT_BIND_PORT = 14525;
  static constexpr char DEFAULT_REMOTE_HOST[] = "localhost";
  static constexpr uint16_t DEFAULT_REMOTE_PORT = 14520;

  struct Buffer
  {
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    size_t len;
    size_t pos;

    Buffer() : len(0), pos(0) {}

    Buffer(const uint8_t *src, size_t len) : len(len), pos(0)
    {
      assert(len <= MAVLINK_MAX_PACKET_LEN); //! \todo Do something less catastrophic here
      memcpy(data, src, len);
    }

    const uint8_t * dpos() const { return data + pos; }
    size_t nbytes() const { return len - pos; }
  };

  typedef std::lock_guard<std::recursive_mutex> mutex_lock;

  void async_read();
  void async_read_end(const boost::system::error_code& error, size_t bytes_transferred);

  void async_write(bool check_write_state);
  void async_write_end(const boost::system::error_code& error, size_t bytes_transferred);

  std::thread io_thread_;
  std::recursive_mutex mutex_;

  uint8_t read_buffer_[MAVLINK_MAX_PACKET_LEN];
  std::list<Buffer*> read_queue_;

  std::list<Buffer*> write_queue_;
  bool write_in_progress_;

  std::string bind_host_;
  uint16_t bind_port_;

  std::string remote_host_;
  uint16_t remote_port_;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint bind_endpoint_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_UDP_BOARD_H
