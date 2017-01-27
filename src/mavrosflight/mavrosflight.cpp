/**
 * \file mavrosflight.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Sources:
 * https://gist.github.com/yoggy/3323808
 */

#include "mavrosflight/mavrosflight.h"

#include <ros/ros.h>

namespace mavrosflight
{
using boost::asio::serial_port_base;

MavROSflight::MavROSflight(std::string port, int baud_rate, uint8_t sysid /* = 1 */, uint8_t compid /* = 50 */)
  : serial(port, baud_rate), param(&serial), time(&serial), sysid_(sysid), compid_(compid)
{
}

MavROSflight::~MavROSflight()
{
}

}  // namespace mavrosflight
