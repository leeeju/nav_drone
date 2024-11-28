#pragma once

#include <cmath>

namespace nav2_drone_util {

/**
 * @brief Calculate the angle between a line defined by two points and the coordinate axes.
 * Result is in radians.
 */
inline double angle(const double x1, const double y1, const double x2, const double y2)
{
  return std::atan2(y2 - y1, x2 - x1);
}

}  // namespace nav2_drone_util
