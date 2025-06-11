#include "nav2_drone_util/angle_utils.hpp"

#include <cmath>
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace nav2_drone_util {

double angle(
  const double x1,
  const double y1,
  const double x2,
  const double y2)
{
  // 두 점이 동일할 경우 유효한 각도가 없으니 0으로 리턴
  if (x1 == x2 && y1 == y2) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav2_drone_util"),
      "The two points are identical, returning 0.0 radians as the angle."
    );
    return 0.0;
  }

  // atan2로 사분면을 모두 처리하여 올바른 라디안 값을 계산
  return std::atan2(y2 - y1, x2 - x1);
}

}  // namespace nav2_drone_util
