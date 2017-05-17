/**
 * \file time_manager.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_TIME_MANAGER_H
#define MAVROSFLIGHT_TIME_MANAGER_H

#include <rosflight/mavrosflight/mavlink_bridge.h>
#include <rosflight/mavrosflight/mavlink_listener_interface.h>
#include <rosflight/mavrosflight/mavlink_serial.h>

#include <ros/ros.h>

#include <cstdlib>
#include <stdint.h>

namespace mavrosflight
{

class TimeManager : MavlinkListenerInterface
{
public:
  TimeManager(MavlinkSerial *serial);

  virtual void handle_mavlink_message(const mavlink_message_t &msg);

  ros::Time get_ros_time_ms(uint32_t boot_ms);
  ros::Time get_ros_time_us(uint32_t boot_us);

private:
  MavlinkSerial *serial_;

  ros::Timer time_sync_timer_;
  void timer_callback(const ros::TimerEvent &event);

  double offset_alpha_;
  int64_t offset_ns_;
  ros::Duration offset_;

  bool initialized_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_TIME_MANAGER_H
