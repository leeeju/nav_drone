#include <cmath>
#include <stdexcept>
#include "nav2_drone_util/costmap_utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp" // For logging warnings

namespace nav2_drone_util {

/**
 * @brief Calculate the elevation (plunge) and azimuth angles between two poses.
 *
 * @param current_pose The current pose in 3D space.
 * @param target_pose The target pose in 3D space.
 * @return A pair of doubles: (plunge in radians, azimuth in radians).
 */
std::pair<double, double> calculate_ez(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & target_pose)
{
  // Calculate differences in coordinates
  double dx = target_pose.pose.position.x - current_pose.pose.position.x;
  double dy = target_pose.pose.position.y - current_pose.pose.position.y;
  double dz = target_pose.pose.position.z - current_pose.pose.position.z;

  // Calculate Euclidean distance between the two points
  double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  // Handle the edge case: if the two poses are identical
  if (distance == 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav2_drone_util"),
      "Identical poses detected in calculate_ez. Returning 0 for both plunge and azimuth."
    );
    return {0.0, 0.0};
  }

  // Direction cosines
  double cosgamma = dz / distance;  // Z-axis direction cosine (used for plunge)

  // Calculate plunge (elevation angle) in radians
  double plunge = std::asin(cosgamma);

  // Prevent division by zero when calculating azimuth
  if (dx == 0.0 && dy == 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav2_drone_util"),
      "Vertical line segment detected in calculate_ez. Azimuth is undefined; returning plunge only."
    );
    return {plunge, 0.0};
  }

  // Calculate azimuth angle (bearing) in radians
  double azimuth = std::atan2(dy, dx);

  return {plunge, azimuth};
}

}  // namespace nav2_drone_util
