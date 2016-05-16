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

MavROSflight::MavROSflight(std::string port, int baud_rate, uint8_t sysid /* = 1 */, uint8_t compid /* = 50 */) :
  io_service_(),
  serial_port_(io_service_),
  sysid_(sysid),
  compid_(compid),
  write_in_progress_(false)
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

void MavROSflight::register_param_value_callback(boost::function<void (char[], float, MAV_PARAM_TYPE)> f)
{
  param_value_callback_ = f;
}

void MavROSflight::unregister_param_value_callback()
{
  param_value_callback_ = NULL;
}

void MavROSflight::register_heartbeat_callback(boost::function<void (void)> f)
{
  heartbeat_callback_ = f;
}

void MavROSflight::unregister_heartbeat_callback()
{
  heartbeat_callback_ = NULL;
}

void MavROSflight::register_imu_callback(boost::function<void (double, double, double, double, double, double)> f)
{
  imu_callback_ = f;
}

void MavROSflight::unregister_imu_callback()
{
  imu_callback_ = NULL;
}

void MavROSflight::send_param_request_list(uint8_t target_system, uint8_t target_component)
{
  mavlink_message_t msg;
  mavlink_msg_param_request_list_pack(sysid_, compid_, &msg, target_system, target_component);
  send_message(msg);
}

void MavROSflight::send_param_set(uint8_t target_system, uint8_t target_component, const char * param_id, int32_t param_value)
{
  mavlink_message_t msg;
  mavlink_msg_param_set_pack(sysid_, compid_, &msg,
                             target_system, target_component, param_id, *(float*) &param_value, MAV_PARAM_TYPE_INT32);
  send_message(msg);
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

void MavROSflight::send_message(const mavlink_message_t &msg)
{
  WriteBuffer *buffer = new WriteBuffer();
  buffer->len = mavlink_msg_to_send_buffer(buffer->data, &msg);
  assert(buffer->len <= MAVLINK_MAX_PACKET_LEN); //! \todo Do something less catastrophic here

  {
    mutex_lock lock(mutex_);
    write_queue_.push_back(buffer);
  }

  do_async_write(true);
}

void MavROSflight::do_async_write(bool check_write_state)
{
  if (check_write_state && write_in_progress_)
    return;

  mutex_lock lock(mutex_);
  if (write_queue_.empty())
    return;

  write_in_progress_ = true;
  WriteBuffer *buffer = write_queue_.front();
  serial_port_.async_write_some(
        boost::asio::buffer(buffer->dpos(), buffer->nbytes()),
        boost::bind(
          &MavROSflight::async_write_end,
          this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));

}

void MavROSflight::async_write_end(const boost::system::error_code &error, std::size_t bytes_transferred)
{
  if (error)
  {
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
    do_async_write(false);
}

void MavROSflight::handle_message()
{
  switch (msg_in_.msgid)
  {
  case MAVLINK_MSG_ID_PARAM_VALUE:
    if (!param_value_callback_.empty())
    {
      mavlink_param_value_t msg;
      mavlink_msg_param_value_decode(&msg_in_, &msg);
      param_value_callback_(msg.param_id, msg.param_value, (MAV_PARAM_TYPE) msg.param_type);
    }
    break;
  case MAVLINK_MSG_ID_HEARTBEAT:
    if (!heartbeat_callback_.empty())
      heartbeat_callback_();
    break;
  case MAVLINK_MSG_ID_SMALL_IMU:
    if (!imu_callback_.empty())
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
