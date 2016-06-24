/**
 * \file mavrosflight.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVROSFLIGHT_H
#define MAVROSFLIGHT_MAVROSFLIGHT_H

#include <mavrosflight/mavlink_bridge.h>
#include <mavrosflight/mavlink_listener_interface.h>
#include <mavrosflight/mavlink_serial.h>

#include <boost/function.hpp>

#include <stdint.h>
#include <string>

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

  // send functions
  void send_param_request_list(uint8_t target_system, uint8_t target_component = MAV_COMP_ID_ALL);
  void send_param_request_read(uint8_t target_system, uint8_t target_component, std::string name);
  void send_param_set(uint8_t target_system, uint8_t target_component, std::string name, int32_t value);
  void send_param_write(uint8_t target_system, uint8_t target_component = MAV_COMP_ID_ALL);
  void send_command(OFFBOARD_CONTROL_MODE mode,
                    OFFBOARD_CONTROL_IGNORE ignore,
                    float value1, float value2, float value3, float value4);

  // public member objects
  MavlinkSerial serial;

private:

  //===========================================================================
  // methods
  //===========================================================================

  /**
   * \brief Saturate a value between lower and upper limits
   * \param value The raw value
   * \param min The lower limit
   * \param max The upper limit
   * \returns The saturated value
   */
  template<class T> inline T saturate(T value, T min, T max)
  {
    return value < min ? min : (value > max ? max : value);
  }

  //===========================================================================
  // member variables
  //===========================================================================

  uint8_t sysid_;
  uint8_t compid_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_MAVROSFLIGHT_H
