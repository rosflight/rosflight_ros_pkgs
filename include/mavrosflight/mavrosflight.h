/**
 * \file mavrosflight.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVROSFLIGHT_H
#define MAVROSFLIGHT_MAVROSFLIGHT_H

#include <mavrosflight/mavlink_bridge.h>
#include <mavrosflight/serial_exception.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <stdint.h>
#include <string>
#include <list>

#define MAVROSFLIGHT_READ_BUF_SIZE 256

namespace mavrosflight
{

class MavROSflight
{
public:

  /**
   * \brief Instantiates the class and begins communication on the specified serial port
   * \param port Name of the serial port (e.g. "/dev/ttyUSB0")
   * \param baud_rate Serial communication baud rate
   */
  MavROSflight(std::string port, int baud_rate, uint8_t sysid = 1, uint8_t compid = 50);

  /**
   * \brief Stops communication and closes the serial port before the object is destroyed
   */
  ~MavROSflight();

  /**
   * \brief Stops communcation and closes the serial port
   */
  void close();

  void register_heartbeat_callback(boost::function<void (void)> f);
  void unregister_heartbeat_callback();

  void register_imu_callback(boost::function<void (double, double, double, double, double, double)> f);
  void unregister_imu_callback();

private:

  //===========================================================================
  // definitions
  //===========================================================================

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

    uint8_t * dpos()
    {
      return data + pos;
    }

    size_t nbytes()
    {
      return len - pos;
    }
  };

  /**
   * \brief Convenience typedef for mutex lock
   */
  typedef boost::mutex::scoped_lock mutex_lock;

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

  void send_message(const mavlink_message_t &msg);
  void do_async_write(bool check_write_state);
  void async_write_end(const boost::system::error_code& error, size_t bytes_transferred);

  /**
   * \brief Handle a received mavlink message
   */
  void handle_message();

  //===========================================================================
  // member variables
  //===========================================================================

  boost::asio::io_service io_service_; //!< boost io service provider
  boost::asio::serial_port serial_port_; //!< boost serial port object
  boost::thread io_thread_; //!< thread on which the io service runs
  boost::mutex mutex_; //!< mutex for threadsafe operation

  uint8_t sysid_;
  uint8_t compid_;

  uint8_t read_buf_raw_[MAVROSFLIGHT_READ_BUF_SIZE];

  mavlink_message_t msg_in_;
  mavlink_status_t status_in_;

  bool heartbeat_callback_registered_;
  boost::function<void (void)> heartbeat_callback_;

  bool imu_callback_registered_;
  boost::function<void (double, double, double, double, double, double)> imu_callback_;

  std::list<WriteBuffer*> write_queue_; //!< queue of buffers to be written to the serial port
  bool write_in_progress_; //!< flag for whether async_write is already running
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_MAVROSFLIGHT_H
