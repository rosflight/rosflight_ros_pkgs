/*
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
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
 * \file mavlink_comm.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <rosflight/mavrosflight/mavlink_comm.h>

namespace mavrosflight
{

using boost::asio::serial_port_base;

MavlinkComm::MavlinkComm() :
  io_service_(),
  write_in_progress_(false)
{
}

MavlinkComm::~MavlinkComm()
{
}

void MavlinkComm::open()
{
  // open the port
  do_open();

  // start reading from the port
  async_read();
  io_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &this->io_service_));
}

void MavlinkComm::close()
{
  mutex_lock lock(mutex_);

  io_service_.stop();
  do_close();

  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void MavlinkComm::register_mavlink_listener(MavlinkListenerInterface * const listener)
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

void MavlinkComm::unregister_mavlink_listener(MavlinkListenerInterface * const listener)
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

void MavlinkComm::async_read()
{
  if (!is_open()) return;

  do_async_read(
        boost::asio::buffer(read_buf_raw_, MAVLINK_SERIAL_READ_BUF_SIZE),
        boost::bind(
          &MavlinkComm::async_read_end,
          this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}

void MavlinkComm::async_read_end(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (!is_open()) return;

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

  async_read();
}

void MavlinkComm::send_message(const mavlink_message_t &msg)
{
  WriteBuffer *buffer = new WriteBuffer();
  buffer->len = mavlink_msg_to_send_buffer(buffer->data, &msg);
  assert(buffer->len <= MAVLINK_MAX_PACKET_LEN); //! \todo Do something less catastrophic here

  {
    mutex_lock lock(mutex_);
    write_queue_.push_back(buffer);
  }

  async_write(true);
}

void MavlinkComm::async_write(bool check_write_state)
{
  if (check_write_state && write_in_progress_)
    return;

  mutex_lock lock(mutex_);
  if (write_queue_.empty())
    return;

  write_in_progress_ = true;
  WriteBuffer *buffer = write_queue_.front();
  do_async_write(
        boost::asio::buffer(buffer->dpos(), buffer->nbytes()),
        boost::bind(
          &MavlinkComm::async_write_end,
          this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));

}

void MavlinkComm::async_write_end(const boost::system::error_code &error, std::size_t bytes_transferred)
{
  if (error)
  {
    std::cerr << error.message() << std::endl;
    close();
    return;
  }

  mutex_lock lock(mutex_);
  if (write_queue_.empty())
  {
    write_in_progress_ = false;
    return;
  }

  WriteBuffer *buffer = write_queue_.front();
  buffer->pos += bytes_transferred;
  if (buffer->nbytes() == 0)
  {
    write_queue_.pop_front();
    delete buffer;
  }

  if (write_queue_.empty())
    write_in_progress_ = false;
  else
    async_write(false);
}

} // namespace mavrosflight
