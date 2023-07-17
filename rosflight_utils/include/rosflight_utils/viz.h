#pragma once

#include <cmath>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/attitude.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rosflight_utils
{
class Viz : public rclcpp::Node
{
public:
  Viz();

private:
  // Magnetometer visualization pubs and subs
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mag_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pts_pub_;

  // Attitude visualization pubs and subs
  rclcpp::Subscription<rosflight_msgs::msg::Attitude>::SharedPtr att_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // Variables
  std::vector<geometry_msgs::msg::Point> pts_list_;
  int mag_throttle_, mag_skip_;
  std::string fixed_frame_ = "fixed_frame";

  // Functions
  void magCallback(const sensor_msgs::msg::MagneticField::ConstSharedPtr& msg);
  void attCallback(const rosflight_msgs::msg::Attitude::ConstSharedPtr& msg);
};

} // namespace rosflight_utils
