#pragma once

#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_drone_util/angle_utils.hpp"
#include "nav2_drone_util/geometry_utils.hpp"

namespace nav2_drone_util
{

/**
 * @brief Get the current pose of the robot in a given frame
 * @param[out] global_pose Output pose transformed into the global frame
 * @param[in] tf_buffer TF buffer for performing transformations
 * @param[in] global_frame Frame to transform the pose into (default: "map")
 * @param[in] robot_frame Frame to transform from (default: "base_link")
 * @param[in] transform_timeout Timeout for the transformation (default: 0.1s)
 * @param[in] stamp Optional timestamp for the transform (default: current time)
 * @return True if the transformation was successful, false otherwise
 */
bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer,
  const std::string global_frame = "map",
  const std::string robot_frame = "base_link",
  const double transform_timeout = 0.1,
  const rclcpp::Time stamp = rclcpp::Time());

/**
 * @brief Get the current pose of the robot as individual components
 * @param[out] x X position of the robot
 * @param[out] y Y position of the robot
 * @param[out] z Z position of the robot
 * @param[out] yaw Yaw orientation of the robot in radians
 * @param[in] tf_buffer TF buffer for performing transformations
 * @param[in] global_frame Frame to transform the pose into (default: "map")
 * @param[in] robot_frame Frame to transform from (default: "base_link")
 * @param[in] transform_timeout Timeout for the transformation (default: 0.1s)
 * @param[in] stamp Optional timestamp for the transform (default: current time)
 * @return True if the transformation was successful, false otherwise
 */
bool getCurrentPose(
  double & x, double & y, double & z, double & yaw,
  tf2_ros::Buffer & tf_buffer,
  const std::string global_frame = "map",
  const std::string robot_frame = "base_link",
  const double transform_timeout = 0.1,
  const rclcpp::Time stamp = rclcpp::Time());

/**
 * @brief Transform a given pose into a target frame
 * @param[in] input_pose Pose to transform
 * @param[out] transformed_pose Output pose in the target frame
 * @param[in] tf_buffer TF buffer for performing transformations
 * @param[in] target_frame Frame to transform the pose into
 * @param[in] transform_timeout Timeout for the transformation (default: 0.1s)
 * @return True if the transformation was successful, false otherwise
 */
bool transformPoseInTargetFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose,
  tf2_ros::Buffer & tf_buffer,
  const std::string target_frame,
  const double transform_timeout = 0.1);

/**
 * @brief Get the yaw angle (rotation around Z-axis) from a quaternion
 * @param orientation Quaternion representing the orientation
 * @return Yaw angle in radians
 */
inline double getYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  return tf2::getYaw(orientation);
}

/**
 * @brief Overloaded function to get yaw from a PoseStamped
 * @param pose PoseStamped containing the orientation
 * @return Yaw angle in radians
 */
inline double getYaw(const geometry_msgs::msg::PoseStamped & pose)
{
  return getYaw(pose.pose.orientation);
}

/**
 * @brief Overloaded function to get yaw from a Pose
 * @param pose Pose containing the orientation
 * @return Yaw angle in radians
 */
inline double getYaw(const geometry_msgs::msg::Pose & pose)
{
  return getYaw(pose.orientation);
}

/**
 * @brief Find the index of the closest goal in a path to the given pose
 * @param pose Current pose of the robot
 * @param path Path containing a series of poses
 * @return Index of the closest goal pose in the path
 */
size_t find_closest_goal_idx(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path);

}  // namespace nav_drone_util
