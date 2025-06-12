#include "nav2_drone_util/angle_utils.hpp"
#include <cmath>
#include "rclcpp/logging.hpp"

namespace nav2_drone_util
{
double calculateAngle(double x1, double y1, double x2, double y2)
{
  if (x1 == x2 && y1 == y2) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav2_drone_util::angle_utils"),
      "The two points are identical, returning 0.0 radians as the angle."
    );
    return 0.0;
  }

  // atan2로 사분면을 모두 처리하여 올바른 라디안 값을 계산
  return std::atan2(y2 - y1, x2 - x1);
}

}  // namespace nav2_drone_util
