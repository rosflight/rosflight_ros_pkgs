/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2017 Devon and Parker Lusk, BYU MAGICC Lab.
 * Copyright (c) 2023 Brandon Sutherland, AeroVironment Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rosflight_utils/viz.hpp>

namespace rosflight_utils
{
Viz::Viz() : Node("viz_node")
{
  // retrieve params

  // initialize variables
  mag_skip_ = 20;
  mag_throttle_ = 0;

  // Magnetometer visualization
  mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
    "/magnetometer", 1, std::bind(&Viz::magCallback, this, std::placeholders::_1));
  mag_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("viz/magnetometer", 1);
  pts_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("viz/cloud", 1);

  // Attitude visualization
  att_sub_ = this->create_subscription<rosflight_msgs::msg::Attitude>(
    "/attitude", 1, std::bind(&Viz::attCallback, this, std::placeholders::_1));
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("viz/attitude", 1);
}

void Viz::magCallback(const sensor_msgs::msg::MagneticField::ConstSharedPtr & msg)
{
  if (mag_throttle_ > mag_skip_) {
    // unpack message
    double x = msg->magnetic_field.x;
    double y = msg->magnetic_field.y;
    double z = msg->magnetic_field.z;

    // get euler angles from vector (assume no roll)
    double yaw = atan2(y, x);
    double pitch = atan2(-z, sqrt(x * x + y * y));

    // convert to body quaternion and rotation into the vehicle frame
    tf2::Quaternion q_v;
    q_v.setRPY(0, pitch, yaw);

    // pack data into pose message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = fixed_frame_;
    pose_msg.pose.position.x = 0;
    pose_msg.pose.position.y = 0;
    pose_msg.pose.position.z = 0;
    pose_msg.pose.orientation = tf2::toMsg(q_v);

    // Publish the messages
    mag_pub_->publish(pose_msg);

    // MEASUREMENT CLOUD //

    // store the current measurement
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    pts_list_.push_back(p);

    // begin packing marker message
    visualization_msgs::msg::Marker pts_msg;
    pts_msg.header.frame_id = fixed_frame_;
    pts_msg.header.stamp = msg->header.stamp;
    pts_msg.type = visualization_msgs::msg::Marker::POINTS;
    pts_msg.action = visualization_msgs::msg::Marker::ADD;

    // set points style
    pts_msg.scale.x = 0.1;
    pts_msg.scale.y = pts_msg.scale.x;
    pts_msg.scale.z = pts_msg.scale.x;
    pts_msg.color.a = 1.0;
    pts_msg.color.r = 0.0;
    pts_msg.color.g = 1.0;
    pts_msg.color.b = 0.0;

    for (const auto & item : pts_list_) { pts_msg.points.push_back(item); }

    // publish point cloud
    pts_pub_->publish(pts_msg);
  }
  mag_throttle_++;
}

void Viz::attCallback(const rosflight_msgs::msg::Attitude::ConstSharedPtr & msg)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = fixed_frame_;
  pose.pose.orientation = msg->attitude;

  pose_pub_->publish(pose);
}

} // namespace rosflight_utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosflight_utils::Viz>());
  rclcpp::shutdown();
  return 0;
}
