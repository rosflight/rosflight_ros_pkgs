/**
 * \file mavrosflight.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Sources:
 * https://gist.github.com/yoggy/3323808
 */

#include <mavrosflight/mavlink_serial.h>

#include <boost/thread.hpp>
#include <ros/ros.h>

#include <iostream>

using namespace std;

namespace mavrosflight
{

using boost::asio::serial_port_base;

MavlinkSerial::MavlinkSerial(std::string port, int baud_rate) :
  io_service_(),
  serial_port_(io_service_),
  write_in_progress_(false),
  write_queue_(100)
{
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

MavlinkSerial::~MavlinkSerial()
{
  close();
}

void MavlinkSerial::register_mavlink_listener(MavlinkListenerInterface * const listener)
{
  if (listener == NULL)
    return;

  bool already_registered = false;
  for (int i = 0; i < listeners_.size(); i++)
  {
    if (listener == listeners_[i])
    {
      already_registered = true;
      break;
    }
  }

  if (!already_registered)
    listeners_.push_back(listener);
}

void MavlinkSerial::unregister_mavlink_listener(MavlinkListenerInterface * const listener)
{
  if (listener == NULL)
    return;

  for (int i = 0; i < listeners_.size(); i++)
  {
    if (listener == listeners_[i])
    {
      listeners_.erase(listeners_.begin() + i);
      i--;
    }
  }
}

void MavlinkSerial::close()
{
  io_service_.stop();
  serial_port_.close();

  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void MavlinkSerial::do_async_read()
{
  if (!serial_port_.is_open()) return;

  serial_port_.async_read_some(
        boost::asio::buffer(read_buf_raw_, MAVLINK_SERIAL_READ_BUF_SIZE),
        boost::bind(
          &MavlinkSerial::async_read_end,
          this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}

void MavlinkSerial::async_read_end(const boost::system::error_code &error, size_t bytes_transferred)
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
      for (int i = 0; i < listeners_.size(); i++)
      {
        listeners_[i]->handle_mavlink_message(msg_in_);
      }
    }
  }

  do_async_read();
}

void MavlinkSerial::send_message(const mavlink_message_t &msg)
{
  WriteBuffer *buffer = new WriteBuffer();
  buffer->len = mavlink_msg_to_send_buffer(buffer->data, &msg);
  assert(buffer->len <= MAVLINK_MAX_PACKET_LEN); //! \todo Do something less catastrophic here
  write_queue_.push(buffer);
  do_async_write(true);
}

void MavlinkSerial::do_async_write(bool check_write_state)
{
  if (check_write_state && write_in_progress_)
    return;

  if (write_queue_.empty())
    return;
  write_in_progress_ = true;
  WriteBuffer *buffer;
  write_queue_.pop(buffer);
  serial_port_.async_write_some(
        boost::asio::buffer(buffer->dpos(), buffer->nbytes()),
        boost::bind(
          &MavlinkSerial::async_write_end,
          this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}

void MavlinkSerial::async_write_end(const boost::system::error_code &error, std::size_t bytes_transferred)
{
  if (error)
  {
    close();
    return;
  }
  else if (write_queue_.empty())
  {
    write_in_progress_ = false;
  }
  else
  {
    do_async_write(false);
  }
}

} // namespace mavrosflight
