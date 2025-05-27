/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2024 Jacob Moore
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

#include "standalone_viz_transcriber.hpp"

namespace rosflight_sim
{

RvizPublisher::RvizPublisher()
    : Node("standalone_viz_transcriber")
{
  declare_parameters();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&RvizPublisher::parameters_callback, this, std::placeholders::_1));

  rclcpp::QoS qos_transient_local_20_(20);
  qos_transient_local_20_.transient_local();
  rviz_wp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/waypoint",
                                                                         qos_transient_local_20_);
  rviz_mesh_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/mesh", 5);
  rviz_aircraft_path_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("rviz/mesh_path", 5);

  vehicle_state_sub_ = this->create_subscription<rosflight_msgs::msg::SimState>(
    "sim/truth_state", 10, std::bind(&RvizPublisher::state_update_callback, this, std::placeholders::_1));

  aircraft_tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Initialize aircraft
  aircraft_.header.frame_id = "stl_frame";
  aircraft_.ns = "vehicle";
  aircraft_.id = 0;
  aircraft_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  aircraft_.mesh_resource = "package://rosflight_sim/" + this->get_parameter("sim_aircraft_file").as_string();
  aircraft_.mesh_use_embedded_materials = false;
  aircraft_.action = visualization_msgs::msg::Marker::ADD;
  aircraft_.pose.position.x = 0.0;
  aircraft_.pose.position.y = 0.0;
  aircraft_.pose.position.z = 0.0;
  aircraft_.pose.orientation.x = 0.0;
  aircraft_.pose.orientation.y = 0.0;
  aircraft_.pose.orientation.z = 0.0;
  aircraft_.pose.orientation.w = 1.0;
  aircraft_.scale.x = this->get_parameter("aircraft_scale").as_double();
  aircraft_.scale.y = this->get_parameter("aircraft_scale").as_double();
  aircraft_.scale.z = this->get_parameter("aircraft_scale").as_double();
  aircraft_.color.r = 0.67f;
  aircraft_.color.g = 0.67f;
  aircraft_.color.b = 0.67f;
  aircraft_.color.a = 1.0;
  rviz_mesh_pub_->publish(aircraft_);

  i_ = 0;
}

void RvizPublisher::declare_parameters()
{
  this->declare_parameter("sim_aircraft_file", "common_resource/skyhunter.dae");
  this->declare_parameter("aircraft_scale", 5.0);
  this->declare_parameter("path_publish_modulo", 10);
  this->declare_parameter("max_path_history", 10000);
}

rcl_interfaces::msg::SetParametersResult
RvizPublisher::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (auto param : parameters) {
    if (param.get_name() == "aircraft_scale") {
      aircraft_.scale.x = param.as_double();
      aircraft_.scale.y = param.as_double();
      aircraft_.scale.z = param.as_double();
    }
  }

  return result;
}

void RvizPublisher::update_aircraft_history()
{
  rclcpp::Time now = this->get_clock()->now();
  aircraft_history_.header.stamp = now;
  aircraft_history_.header.frame_id = "NED";
  aircraft_history_.ns = "vehicle_path";
  aircraft_history_.id = 0;
  aircraft_history_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  aircraft_history_.action = visualization_msgs::msg::Marker::ADD;
  aircraft_history_.scale.x = 1.0;
  aircraft_history_.scale.y = 1.0;
  aircraft_history_.scale.z = 1.0;
  aircraft_history_.color.r = 0.0f;
  aircraft_history_.color.g = 0.0f;
  aircraft_history_.color.b = 0.0f;
  aircraft_history_.color.a = 1.0;
  aircraft_history_.points = aircraft_history_points_;

  // Restrict length of history
  if (aircraft_history_points_.size() > (uint64_t) this->get_parameter("max_path_history").as_int()) {
    aircraft_history_points_.erase(aircraft_history_points_.begin());
  }
}

void RvizPublisher::update_mesh()
{
  rclcpp::Time now = this->get_clock()->now();
  aircraft_.header.stamp = now;

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "NED";
  t.child_frame_id = "aircraft_body";
  t.transform.translation.x = vehicle_state_.pose.position.x;
  t.transform.translation.y = vehicle_state_.pose.position.y;
  t.transform.translation.z = vehicle_state_.pose.position.z;

  t.transform.rotation.x = vehicle_state_.pose.orientation.x;
  t.transform.rotation.y = vehicle_state_.pose.orientation.y;
  t.transform.rotation.z = vehicle_state_.pose.orientation.z;
  t.transform.rotation.w = vehicle_state_.pose.orientation.w;

  // Update aircraft history
  if (i_ % this->get_parameter("path_publish_modulo").as_int() == 0) {
    geometry_msgs::msg::Point new_p;
    new_p.x = vehicle_state_.pose.position.x;
    new_p.y = vehicle_state_.pose.position.y;
    new_p.z = vehicle_state_.pose.position.z;
    aircraft_history_points_.push_back(new_p);
    update_aircraft_history();

    rviz_aircraft_path_pub_->publish(aircraft_history_);
  }

  aircraft_tf2_broadcaster_->sendTransform(t);
  rviz_mesh_pub_->publish(aircraft_);
}

void RvizPublisher::state_update_callback(const rosflight_msgs::msg::SimState & msg)
{
  vehicle_state_ = msg;
  update_mesh();
}

} // namespace rosflight_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosflight_sim::RvizPublisher>();

  rclcpp::spin(node);

  return 0;
}
