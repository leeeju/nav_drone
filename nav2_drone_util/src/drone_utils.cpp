#include "nav2_drone_util/tf_help.hpp"  // Ensure this is included
#include "nav2_drone_util/drone_utils.hpp"
#include "rclcpp/logger.hpp"

namespace nav2_drone_util
{

bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer,
  const std::string & global_frame,
  const std::string & drone_frame,
  double transform_timeout,
  rclcpp::Time stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp == rclcpp::Time() ? tf_buffer.get_clock()->now() : stamp;
  pose.header.frame_id = drone_frame;
  pose.pose.orientation.w = 1.0;

  try {
    tf_buffer.transform(
      pose, global_pose, global_frame,
      tf2::durationFromSec(transform_timeout));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("nav2_drone_util"),
      "Failed to get current pose from %s to %s: %s",
      drone_frame.c_str(), global_frame.c_str(), ex.what());
    return false;
  }
}

bool getCurrentPose(
  double & x, double & y, double & z, double & yaw,
  tf2_ros::Buffer & tf_buffer,
  const std::string & global_frame,
  const std::string & drone_frame,
  double transform_timeout,
  rclcpp::Time stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  if (!getCurrentPose(pose, tf_buffer, global_frame, drone_frame, transform_timeout, stamp)) {
    return false;
  }

  x = pose.pose.position.x;
  y = pose.pose.position.y;
  z = pose.pose.position.z;
  yaw = tf2::getYaw(pose.pose.orientation);
  return true;
}

}  // namespace nav2_drone_util
