#include <rosflight_utils/viz_mag.h>

namespace rosflight_utils
{

VizMag::VizMag() :
  nh_private_("~")
{
  // retrieve params
  
  // initialize variables
  mag_sum_ = 0;
  mag_skip_ = 20;
  mag_count_ = 0;
  mag_throttle_ = 0;

  // Set up Publishers and Subscribers
  mag_sub_ = nh_.subscribe("/magnetometer", 1, &VizMag::magCallback, this);
  
  mag_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "viz/magnetometer", 1 );
  pts_pub_ = nh_.advertise<visualization_msgs::Marker>( "viz/cloud", 1);
}

void VizMag::magCallback(const sensor_msgs::MagneticFieldConstPtr &msg)
{  
  if (mag_throttle_ > mag_skip_)
  {
    // unpack message
    double x = msg->magnetic_field.x;
    double y = msg->magnetic_field.y;
    double z = msg->magnetic_field.z;

    // get euler angles from vector (assume no roll)
    double yaw = atan2(y, x);
    double pitch = atan2(-z, sqrt(x*x + y*y));

    // convert to body quaternion and rotation into the vehicle frame
    tf::Quaternion q_v = tf::createQuaternionFromRPY(0, pitch, yaw);

    // pack data into pose message
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = "fixed_frame";
    pose_msg.pose.position.x = 0;
    pose_msg.pose.position.y = 0;
    pose_msg.pose.position.z = 0;
    tf::quaternionTFToMsg(q_v, pose_msg.pose.orientation);

    // Publish the messages
    mag_pub_.publish(pose_msg);


    // MEASUREMENT CLOUD //

    // store the current measurement
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    pts_list_.push_back(p);

    // begin packing marker message
    visualization_msgs::Marker pts_msg;
    pts_msg.header.frame_id = "fixed_frame";
    pts_msg.header.stamp = msg->header.stamp;
    pts_msg.type = visualization_msgs::Marker::POINTS;
    pts_msg.action = visualization_msgs::Marker::ADD;

    // set points style
    pts_msg.scale.x = 0.1;
    pts_msg.scale.y = pts_msg.scale.x;
    pts_msg.scale.z = pts_msg.scale.x;
    pts_msg.color.a = 1.0;
    pts_msg.color.r = 0.0;
    pts_msg.color.g = 1.0;
    pts_msg.color.b = 0.0;

    for (uint32_t i = 0; i < pts_list_.size(); ++i)
    {
        pts_msg.points.push_back(pts_list_[i]);
    }

    // publish point cloud
    pts_pub_.publish(pts_msg);
  }
  mag_throttle_++;
}


} // namespace rosflight

int main(int argc, char** argv)
{
  ros::init(argc, argv, "viz_mag_node");
  rosflight::VizMag thing;
  ros::spin();
  return 0;
}

