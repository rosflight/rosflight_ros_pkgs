/**
 * \file mavrosflight.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Sources:
 * https://gist.github.com/yoggy/3323808
 */

#include "mavrosflight/mavrosflight.h"

#include <boost/thread.hpp>

namespace mavrosflight
{

using boost::asio::serial_port_base;

MavROSflight::MavROSflight(std::string port, int baud_rate) :
  io_service_(),
  serial_port_(io_service_),
  heartbeat_callback_registered_(false),
  imu_callback_registered_(false)
{
  // setup mavlink
//  mavlink_system.sysid = 1;
//  mavlink_system.compid = 0;

  // setup serial port
  try
  {
    serial_port_.open(port);
    serial_port_.set_option(serial_port_base::baud_rate(baud_rate));
    serial_port_.set_option(serial_port_base::character_size(8));
    serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
  }
  catch (boost::system::system_error e)
  {
    throw SerialException(e);
  }

  // start reading from serial port
  do_async_read();
  io_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &this->io_service_));
}

MavROSflight::~MavROSflight()
{
  close();
}

void MavROSflight::close()
{
  mutex_lock lock(mutex_);

  io_service_.stop();
  serial_port_.close();

  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void MavROSflight::register_heartbeat_callback(boost::function<void (void)> f)
{
  heartbeat_callback_ = f;
  heartbeat_callback_registered_ = true;
}

void MavROSflight::unregister_heartbeat_callback()
{
  heartbeat_callback_registered_ = false;
}

void MavROSflight::register_imu_callback(boost::function<void (double, double, double, double, double, double)> f)
{
  imu_callback_ = f;
  imu_callback_registered_ = true;
}

void MavROSflight::unregister_imu_callback()
{
  imu_callback_registered_ = false;
}

void MavROSflight::do_async_read()
{
  if (!serial_port_.is_open()) return;

  serial_port_.async_read_some(
        boost::asio::buffer(read_buf_raw_, MAVROSFLIGHT_READ_BUF_SIZE),
        boost::bind(
          &MavROSflight::async_read_end,
          this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}

void MavROSflight::async_read_end(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (!serial_port_.is_open()) return;

  if (error)
  {
    close();
    return;
  }

  for (int i = 0; i < bytes_transferred; i++)
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, read_buf_raw_[i], &msg_in_, &status_in_))
    {
      handle_message();
    }
  }

  do_async_read();
}

void MavROSflight::handle_message()
{
  switch (msg_in_.msgid)
  {
  case MAVLINK_MSG_ID_HEARTBEAT:
    if (heartbeat_callback_registered_)
      heartbeat_callback_();
    break;
  case MAVLINK_MSG_ID_SMALL_IMU:
    if (imu_callback_registered_)
    {
      mavlink_small_imu_t msg;
      mavlink_msg_small_imu_decode(&msg_in_, &msg);

      //TODO convert units
      imu_callback_(msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro);
    }
    break;
  default:
    break;
  }
}

} // namespace mavrosflight
