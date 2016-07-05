/**
 * \file mavlink_serial.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVLINK_SERIAL_H
#define MAVROSFLIGHT_MAVLINK_SERIAL_H

#include <mavrosflight/mavlink_bridge.h>
#include <mavrosflight/mavlink_listener_interface.h>
#include <mavrosflight/serial_exception.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <list>
#include <string>
#include <vector>

#include <stdint.h>

#define MAVLINK_SERIAL_READ_BUF_SIZE 256

namespace mavrosflight
{

class MavlinkSerial
{
public:

  /**
   * \brief Instantiates the class and begins communication on the specified serial port
   * \param port Name of the serial port (e.g. "/dev/ttyUSB0")
   * \param baud_rate Serial communication baud rate
   */
  MavlinkSerial(std::string port, int baud_rate);

  /**
   * \brief Stops communication and closes the serial port before the object is destroyed
   */
  ~MavlinkSerial();

  /**
   * \brief Register a listener for mavlink messages
   * \param listener Pointer to an object that implements the MavlinkListenerInterface interface
   */
  void register_mavlink_listener(MavlinkListenerInterface * const listener);

  /**
   * \brief Unregister a listener for mavlink messages
   * \param listener Pointer to an object that implements the MavlinkListenerInterface interface
   */
  void unregister_mavlink_listener(MavlinkListenerInterface * const listener);

  /**
   * \brief Send a mavlink message
   * \param msg The message to send
   */
  void send_message(const mavlink_message_t &msg);

private:

  //===========================================================================
  // definitions
  //===========================================================================

  /**
   * \brief Struct for buffering the contents of a mavlink message
   */
  struct WriteBuffer
  {
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    size_t len;
    size_t pos;

    WriteBuffer() : len(0), pos(0) {}

    WriteBuffer(const uint8_t * buf, uint16_t len) : len(len), pos(0)
    {
      assert(len <= MAVLINK_MAX_PACKET_LEN); //! \todo Do something less catastrophic here
      memcpy(data, buf, len);
    }

    uint8_t * dpos() { return data + pos; }

    size_t nbytes() { return len - pos; }
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
  void do_async_read();

  /**
   * \brief Handler for end of asynchronous read operation
   * \param error Error code
   * \param bytes_transferred Number of bytes received
   */
  void async_read_end(const boost::system::error_code& error, size_t bytes_transferred);

  /**
   * \brief Initialize an asynchronous write operation
   * \param check_write_state If true, only start another write operation if a write sequence is not already running
   */
  void do_async_write(bool check_write_state);

  /**
   * \brief Handler for end of asynchronous write operation
   * \param error Error code
   * \param bytes_transferred Number of bytes sent
   */
  void async_write_end(const boost::system::error_code& error, size_t bytes_transferred);

  /**
   * \brief Stops communication and closes the serial port
   */
  void close();

  //===========================================================================
  // member variables
  //===========================================================================

  std::vector<MavlinkListenerInterface*> listeners_; //!< listeners for mavlink messages

  boost::asio::io_service io_service_; //!< boost io service provider
  boost::asio::serial_port serial_port_; //!< boost serial port object
  boost::thread io_thread_; //!< thread on which the io service runs
  boost::recursive_mutex mutex_; //!< mutex for threadsafe operation

  uint8_t sysid_;
  uint8_t compid_;

  uint8_t read_buf_raw_[MAVLINK_SERIAL_READ_BUF_SIZE];

  mavlink_message_t msg_in_;
  mavlink_status_t status_in_;

  std::list<WriteBuffer*> write_queue_; //!< queue of buffers to be written to the serial port
  bool write_in_progress_; //!< flag for whether async_write is already running
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_MAVLINK_SERIAL_H
