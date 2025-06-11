#pragma once

#include <string>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // Updated include
#include "tf2/utils.h"  // Include the correct header for getYaw
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_drone_util
{
// Overloaded function declarations
bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer,
  const std::string & global_frame = "map",
  const std::string & drone_frame = "base_link",
  double transform_timeout = 0.1,
  rclcpp::Time stamp = rclcpp::Time());

bool getCurrentPose(
  double & x, double & y, double & z, double & yaw,
  tf2_ros::Buffer & tf_buffer,
  const std::string & global_frame = "map",
  const std::string & drone_frame = "base_link",
  double transform_timeout = 0.1,
  rclcpp::Time stamp = rclcpp::Time());

inline double getYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  return tf2::getYaw(orientation);
}
}  // namespace nav2_drone_util
