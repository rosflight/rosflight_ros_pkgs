#ifndef VIZ_MAG_H
#define VIZ_MAG_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace rosflight_utils
{


class VizMag
{

public:

  VizMag();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber mag_sub_;
  ros::Publisher mag_pub_;
  ros::Publisher pts_pub_;

  // Variables
  tf::Quaternion q_att_;
  std::vector<geometry_msgs::Point> pts_list_;
  double mag_sum_;
  int mag_count_, mag_throttle_, mag_skip_;

  // Functions
  void magCallback(const sensor_msgs::MagneticFieldConstPtr &msg);
};

} // namespace visual_naze

#endif

