#pragma once

#include <string>
#include <memory>
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_drone_util
{
/**
 * @brief Transform a PoseStamped from one frame to another with exception handling.
 *
 * This function transforms a `PoseStamped` message into a specified target frame. If the frames
 * are already identical, it directly copies the input pose to the output pose.
 * Handles potential exceptions that may occur during transformations.
 *
 * @param tf Shared pointer to the TF2 buffer for transformations.
 * @param frame Target frame to transform the pose into.
 * @param in_pose Input pose to be transformed.
 * @param out_pose Reference to store the transformed pose.
 * @param transform_tolerance Tolerance duration for checking valid transform data.
 * @return True if the transformation was successful, false otherwise.
 */
bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance);

}  // namespace nav2_drone_util
