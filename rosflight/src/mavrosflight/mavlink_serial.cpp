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
 * \file mavlink_serial.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <rosflight/mavrosflight/mavlink_serial.h>
#include <rosflight/mavrosflight/serial_exception.h>

namespace mavrosflight
{

using boost::asio::serial_port_base;

MavlinkSerial::MavlinkSerial(std::string port, int baud_rate) :
  MavlinkComm(),
  serial_port_(io_service_),
  port_(port),
  baud_rate_(baud_rate)
{
}

MavlinkSerial::~MavlinkSerial()
{
  do_close();
}

bool MavlinkSerial::is_open()
{
  return serial_port_.is_open();
}

void MavlinkSerial::do_open()
{
  try
  {
    serial_port_.open(port_);
    serial_port_.set_option(serial_port_base::baud_rate(baud_rate_));
    serial_port_.set_option(serial_port_base::character_size(8));
    serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
  }
  catch (boost::system::system_error e)
  {
    throw SerialException(e);
  }
}

void MavlinkSerial::do_close()
{
  serial_port_.close();
}

void MavlinkSerial::do_async_read(const boost::asio::mutable_buffers_1 &buffer, boost::function<void(const boost::system::error_code&, size_t)> handler)
{
  serial_port_.async_read_some(buffer, handler);
}

void MavlinkSerial::do_async_write(const boost::asio::const_buffers_1 &buffer, boost::function<void(const boost::system::error_code&, size_t)> handler)
{
  serial_port_.async_write_some(buffer, handler);
}

} // namespace mavrosflight
