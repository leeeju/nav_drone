#include "nav2_drone_util/angle_utils.hpp"
#include <cmath> // For atan2 and M_PI
#include "rclcpp/rclcpp.hpp"

namespace nav2_drone_util
{

/**
 * @brief Calculate the angle between a line defined by two points and the x-axis.
 *
 * This function calculates the angle in radians between the line segment defined
 * by the points (x1, y1) and (x2, y2) and the positive x-axis. The result is in
 * the range [-PI, PI].
 *
 * @param x1 The x-coordinate of the first point.
 * @param y1 The y-coordinate of the first point.
 * @param x2 The x-coordinate of the second point.
 * @param y2 The y-coordinate of the second point.
 * @return The angle in radians.
 */
double angle(const double x1, const double y1, const double x2, const double y2)
{
  if (x1 == x2 && y1 == y2) {
    // If the points are identical, the angle is undefined.
    // Logging a warning and returning 0 as a safe fallback.
    RCLCPP_WARN(
      rclcpp::get_logger("nav_drone_util"),
      "Identical points detected. Angle calculation is undefined. Returning 0."
    );
    return 0.0;
  }

  // atan2 computes the angle in the correct quadrant, handling all edge cases.
  return std::atan2(y2 - y1, x2 - x1);
}

}  // namespace nav2_drone_util
