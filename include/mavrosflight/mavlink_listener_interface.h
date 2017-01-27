/**
 * \file mavlink_listener_interface.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVLINK_LISTENER_INTERFACE_H
#define MAVROSFLIGHT_MAVLINK_LISTENER_INTERFACE_H

#include <mavrosflight/mavlink_bridge.h>

namespace mavrosflight
{
/**
 * \brief Describes an interface classes can implement to receive and handle mavlink messages
 */
class MavlinkListenerInterface
{
public:
  /**
   * \brief The handler function for mavlink messages to be implemented by derived classes
   * \param msg The mavlink message to handle
   */
  virtual void handle_mavlink_message(const mavlink_message_t &msg) = 0;
};

}  // namespace mavrosflight

#endif  // MAVROSFLIGHT_MAVLINK_LISTENER_INTERFACE_H
