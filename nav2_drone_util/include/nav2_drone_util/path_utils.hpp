#pragma once

#include <vector>
#include <limits>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_drone_util
{
/**
 * @brief Find the index of the closest pose in a path to the given current pose.
 *
 * @param current_pose The current robot pose.
 * @param path A vector of PoseStamped representing the planned path.
 * @return Index of the closest pose in the path.
 */
inline size_t find_closest_goal_idx(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  double min_dist = std::numeric_limits<double>::infinity();
  size_t min_idx = 0;
  for (size_t i = 0; i < path.size(); ++i) {
    const auto & p = path[i].pose.position;
    const auto & c = current_pose.pose.position;
    double dx = p.x - c.x;
    double dy = p.y - c.y;
    double dz = p.z - c.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

/**
 * @brief Calculate the remaining path length from a given start index to the end.
 *
 * @param path A vector of PoseStamped representing the planned path.
 * @param start_idx Index to start calculating distance from.
 * @return Total Euclidean distance along the path from start_idx to the end.
 */
inline double calculate_path_length(
  const std::vector<geometry_msgs::msg::PoseStamped> & path,
  size_t start_idx)
{
  double total_dist = 0.0;
  for (size_t i = start_idx + 1; i < path.size(); ++i) {
    const auto & p0 = path[i - 1].pose.position;
    const auto & p1 = path[i].pose.position;
    double dx = p1.x - p0.x;
    double dy = p1.y - p0.y;
    double dz = p1.z - p0.z;
    total_dist += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return total_dist;
}

}  // namespace nav2_drone_util
