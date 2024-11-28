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
 * @brief Transform a PoseStamped from one frame to another while catching exceptions
 *
 * This function also handles the case where the input and target frames are the same,
 * returning immediately without performing any transformation.
 *
 * @param tf Shared pointer to the TF2 buffer used for transformations
 * @param target_frame The target frame to transform the pose into
 * @param in_pose The input pose to transform
 * @param[out] out_pose The transformed pose
 * @param transform_tolerance Duration for the transformation timeout
 * @return True if the transformation was successful, false otherwise
 */
inline bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string & target_frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance)
{
  try {
    // If the target frame is the same as the input pose's frame, no transformation is needed
    if (in_pose.header.frame_id == target_frame) {
      out_pose = in_pose;
      return true;
    }

    // Perform the transformation
    out_pose = tf->transform(in_pose, target_frame, transform_tolerance.to_chrono<std::chrono::seconds>());
    return true;
  } catch (tf2::TransformException & ex) {
    // Log the exception for debugging purposes
    RCLCPP_WARN(
      rclcpp::get_logger("tf_help"),
      "Failed to transform pose from frame '%s' to frame '%s': %s",
      in_pose.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

}  // namespace nav_drone_util
