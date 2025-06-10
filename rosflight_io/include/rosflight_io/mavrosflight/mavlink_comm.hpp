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
 * \file mavlink_comm.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVLINK_COMM_H
#define MAVROSFLIGHT_MAVLINK_COMM_H

#include <rosflight_io/mavrosflight/mavlink_bridge.hpp>
#include <rosflight_io/mavrosflight/mavlink_listener_interface.hpp>

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

#include <cstdint>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#define MAVLINK_SERIAL_READ_BUF_SIZE 256

namespace mavrosflight
{
class MavlinkComm
{
public:
  /**
   * \brief Instantiates the class and begins communication on the specified serial port
   * \param port Name of the serial port (e.g. "/dev/ttyUSB0")
   * \param baud_rate Serial communication baud rate
   */
  MavlinkComm();

  /**
   * \brief Stops communication and closes the serial port before the object is destroyed
   */
  virtual ~MavlinkComm();

  /**
   * \brief Opens the port and begins communication
   */
  void open();

  /**
   * \brief Stops communication and closes the port
   */
  void close();

  /**
   * \brief Register a listener for mavlink messages
   * \param listener Pointer to an object that implements the MavlinkListenerInterface interface
   */
  void register_mavlink_listener(MavlinkListenerInterface * listener);

  /**
   * \brief Unregister a listener for mavlink messages
   * \param listener Pointer to an object that implements the MavlinkListenerInterface interface
   */
  void unregister_mavlink_listener(MavlinkListenerInterface * listener);

  /**
   * \brief Send a mavlink message
   * \param msg The message to send
   */
  void send_message(const mavlink_message_t & msg);

protected:
  virtual bool is_open() = 0;
  virtual void do_open() = 0;
  virtual void do_close() = 0;
  virtual void
  do_async_read(const boost::asio::mutable_buffers_1 & buffer,
                boost::function<void(const boost::system::error_code &, size_t)> handler) = 0;
  virtual void
  do_async_write(const boost::asio::const_buffers_1 & buffer,
                 boost::function<void(const boost::system::error_code &, size_t)> handler) = 0;

  boost::asio::io_service io_service_; //!< boost io service provider

private:
  //===========================================================================
  // definitions
  //===========================================================================

  /**
   * \brief Struct for buffering the contents of a mavlink message
   */
  struct WriteBuffer
  {
    uint8_t data[MAVLINK_MAX_PACKET_LEN] = {0};
    size_t len;
    size_t pos;

    WriteBuffer()
        : len(0)
        , pos(0)
    {}

    WriteBuffer(const uint8_t * buf, uint16_t len)
        : len(len)
        , pos(0)
    {
      assert(len <= MAVLINK_MAX_PACKET_LEN); //! \todo Do something less catastrophic here
      memcpy(data, buf, len);
    }

    const uint8_t * dpos() const { return data + pos; }

    size_t nbytes() const { return len - pos; }
  };

  /**
   * \brief Convenience typedef for mutex lock
   */
  typedef boost::lock_guard<boost::recursive_mutex> mutex_lock;

  //===========================================================================
  // methods
  //===========================================================================

  /**
   * \brief Initiate an asynchronous read operation
   */
  void async_read();

  /**
   * \brief Handler for end of asynchronous read operation
   * \param error Error code
   * \param bytes_transferred Number of bytes received
   */
  void async_read_end(const boost::system::error_code & error, size_t bytes_transferred);

  /**
   * \brief Initialize an asynchronous write operation
   * \param check_write_state If true, only start another write operation if a write sequence is not already running
   */
  void async_write(bool check_write_state);

  /**
   * \brief Handler for end of asynchronous write operation
   * \param error Error code
   * \param bytes_transferred Number of bytes sent
   */
  void async_write_end(const boost::system::error_code & error, size_t bytes_transferred);

  //===========================================================================
  // member variables
  //===========================================================================

  std::vector<MavlinkListenerInterface *> listeners_; //!< listeners for mavlink messages

  boost::thread io_thread_;      //!< thread on which the io service runs
  boost::recursive_mutex mutex_; //!< mutex for threadsafe operation

  uint8_t read_buf_raw_[MAVLINK_SERIAL_READ_BUF_SIZE];

  mavlink_message_t msg_in_;
  mavlink_status_t status_in_;

  std::list<WriteBuffer *> write_queue_; //!< queue of buffers to be written to the serial port
  bool write_in_progress_;               //!< flag for whether async_write is already running
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_MAVLINK_COMM_H
