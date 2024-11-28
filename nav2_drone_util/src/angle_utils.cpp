#include "nav2_drone_util/angle_utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace nav2_drone_util {

// Calculate the angle between a line defined by two points and the coordinate axes.
// The result is in RADIANS.
double angle(const double x1, const double y1, const double x2, const double y2)
{
  // If the two points are the same, return 0 as there is no valid angle.
  if (x1 == x2 && y1 == y2) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav2_drone_util"),
      "The two points are identical, returning 0.0 radians as the angle."
    );
    return 0.0;
  }

  // Use atan2 to handle all quadrants and return the correct angle in radians.
  return std::atan2(y2 - y1, x2 - x1);
}

}  // namespace nav2_drone_util
