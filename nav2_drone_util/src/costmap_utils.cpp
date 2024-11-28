#include <cmath>
#include <stdexcept> // For exceptions
#include "nav2_drone_util/costmap_utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp" // For RCLCPP_WARN

namespace nav2_drone_util {

/**
 * @brief Calculates elevation (plunge) and azimuth angles between the current pose and the target pose.
 *
 * Uses direction cosines to calculate the azimuth and elevation angles, providing a 3D relationship between two poses.
 *
 * @param current_pose The current pose in 3D space.
 * @param target_pose The target pose in 3D space.
 * @return A pair of doubles: (plunge, azimuth).
 */
std::pair<double, double> calculate_ez(const geometry_msgs::msg::PoseStamped & current_pose,
                                       const geometry_msgs::msg::PoseStamped & target_pose)
{
  // Calculate distance between points
  double dx = target_pose.pose.position.x - current_pose.pose.position.x;
  double dy = target_pose.pose.position.y - current_pose.pose.position.y;
  double dz = target_pose.pose.position.z - current_pose.pose.position.z;
  double distance = std::sqrt(dx * dx + dy * dy + dz * dz); // Euclidean distance in 3D space

  // Handle edge case: identical points (distance = 0)
  if (distance == 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav_drone_util"),
      "Identical poses detected. Returning 0 for both azimuth and plunge."
    );
    return {0.0, 0.0};
  }

  // Direction cosines
  double cosalpha = dx / distance; // X-axis direction cosine
  double cosbeta = dy / distance;  // Y-axis direction cosine
  double cosgamma = dz / distance; // Z-axis direction cosine (plunge)

  // Calculate plunge (elevation angle) in radians
  double plunge = std::asin(cosgamma); // Positive downward if target_pose.z > current_pose.z

  // Calculate azimuth angle (bearing) in radians using atan2 to ensure correct quadrant
  double azimuth = std::atan2(dy, dx);

  return {plunge, azimuth};
}

} // namespace nav2_drone_util
