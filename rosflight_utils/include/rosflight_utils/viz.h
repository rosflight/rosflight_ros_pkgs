#pragma once

#include <math.h>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <rosflight_msgs/Attitude.h>
#include <visualization_msgs/Marker.h>

namespace rosflight_utils
{


class Viz
{
public:
  Viz();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Magnetometer visualization pubs and subs
  ros::Subscriber mag_sub_;
  ros::Publisher mag_pub_;
  ros::Publisher pts_pub_;

  // Attitude visualization pubs and subs
  ros::Subscriber att_sub_;
  ros::Publisher pose_pub_;

  // Variables
  tf::Quaternion q_att_;
  std::vector<geometry_msgs::Point> pts_list_;
  double mag_sum_;
  int mag_count_, mag_throttle_, mag_skip_;
  std::string fixed_frame_ = "fixed_frame";

  // Functions
  void magCallback(const sensor_msgs::MagneticFieldConstPtr &msg);
  void attCallback(const rosflight_msgs::AttitudeConstPtr &msg);
};

} // namespace rosflight_utils
