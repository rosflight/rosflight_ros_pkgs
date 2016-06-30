/**
 * \file mavrosflight.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVROSFLIGHT_H
#define MAVROSFLIGHT_MAVROSFLIGHT_H

#include <mavrosflight/mavlink_bridge.h>
#include <mavrosflight/mavlink_serial.h>
#include <mavrosflight/param_manager.h>
#include <mavrosflight/time_manager.h>

#include <mavrosflight/mavlink_listener_interface.h>
#include <mavrosflight/param_listener_interface.h>

#include <mavrosflight/sensors/differential_pressure.h>
#include <mavrosflight/sensors/imu.h>
#include <mavrosflight/sensors/baro.h>

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

  // public member objects
  MavlinkSerial serial;
  ParamManager param;
  TimeManager time;

private:

  // member variables
  uint8_t sysid_;
  uint8_t compid_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_MAVROSFLIGHT_H
