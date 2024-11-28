#include "nav2_drone_util/robot_utils.hpp"
#include "tf2/utils.h"
#include <limits>
#include <cmath> // For std::abs

namespace nav2_drone_util
{

// Extract yaw from a quaternion
double getYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  return tf2::getYaw(orientation);
}

// Get the robot's current pose in the global frame
bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame,
  const std::string robot_frame, const double transform_timeout,
  const rclcpp::Time stamp)
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  global_pose.header.frame_id = robot_frame;
  global_pose.header.stamp = stamp;

  return transformPoseInTargetFrame(
    global_pose, global_pose, tf_buffer, global_frame, transform_timeout);
}

// Get the robot's current position (x, y, z, yaw)
bool getCurrentPose(
  double &x, double &y, double &z, double &yaw,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame,
  const std::string robot_frame, const double transform_timeout,
  const rclcpp::Time stamp)
{
  geometry_msgs::msg::PoseStamped global_pose;
  if (getCurrentPose(global_pose, tf_buffer, global_frame,
                     robot_frame, transform_timeout, stamp)) {
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;
    z = global_pose.pose.position.z;
    yaw = getYaw(global_pose.pose.orientation);
    return true;
  }
  return false;
}

// Transform a pose into a target frame
bool transformPoseInTargetFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose,
  tf2_ros::Buffer & tf_buffer, const std::string target_frame,
  const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("nav_drone_util");

  if (input_pose.header.frame_id == target_frame) {
    transformed_pose = input_pose;
    return true;
  }

  try {
    transformed_pose = tf_buffer.transform(
      input_pose, target_frame,
      tf2::durationFromSec(transform_timeout));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger, "Failed to transform from %s to %s: %s",
      input_pose.header.frame_id.c_str(), target_frame.c_str(), ex.what());
  }

  return false;
}

// Calculate the length of the remaining path
double calculate_path_length(
  const nav_msgs::msg::Path & path, const size_t current_idx)
{
  if (current_idx >= path.poses.size() - 1) {
    return 0.0;
  }

  double distance = 0.0;
  for (size_t i = current_idx; i < path.poses.size() - 1; ++i) {
    distance += euclidean_distance(path.poses[i], path.poses[i + 1]);
  }

  return distance;
}

// Find the closest pose in the path to the robot's current pose
size_t find_closest_goal_idx(
  const geometry_msgs::msg::PoseStamped & pose, const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("nav_drone_util"), "Path is empty. Returning 0.");
    return 0;
  }

  size_t closest_pose_idx = 0;
  double curr_min_dist = std::numeric_limits<double>::max();

  for (size_t curr_idx = 0; curr_idx < path.poses.size(); ++curr_idx) {
    double curr_dist = euclidean_distance(pose, path.poses[curr_idx]);
    if (curr_dist < curr_min_dist) {
      curr_min_dist = curr_dist;
      closest_pose_idx = curr_idx;
    }
  }

  return closest_pose_idx;
}

}  // namespace nav2_drone_util
