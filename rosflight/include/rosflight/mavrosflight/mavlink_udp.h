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
 * \file mavlink_udp.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVLINK_UDP_H
#define MAVROSFLIGHT_MAVLINK_UDP_H

#include <rosflight/mavrosflight/mavlink_comm.h>

#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <string>

namespace mavrosflight
{

class MavlinkUDP : public MavlinkComm
{
public:

  /**
   * \brief Instantiates the class and begins communication on the specified serial port
   * \param bind_host Host where this node is running
   * \param bind_port Port number for this node
   * \param remote_host Host where the other node is running
   * \param remote_port Port number for the other node
   */
  MavlinkUDP(std::string bind_host, uint16_t bind_port, std::string remote_host, uint16_t remote_port);

  /**
   * \brief Stops communication and closes the serial port before the object is destroyed
   */
  ~MavlinkUDP();

private:

  //===========================================================================
  // methods
  //===========================================================================

  virtual bool is_open();
  virtual void do_open();
  virtual void do_close();
  virtual void do_async_read(const boost::asio::mutable_buffers_1 &buffer, boost::function<void(const boost::system::error_code&, size_t)> handler);
  virtual void do_async_write(const boost::asio::const_buffers_1 &buffer, boost::function<void(const boost::system::error_code&, size_t)> handler);

  //===========================================================================
  // member variables
  //===========================================================================

  std::string bind_host_;
  uint16_t bind_port_;

  std::string remote_host_;
  uint16_t remote_port_;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint bind_endpoint_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_MAVLINK_UDP_H
