#pragma once

#include <cmath>
#include <angles/angles.h>

namespace nav2_drone_util
{
/**
 * @brief Calculate the angle between two points relative to the x-axis.
 *
 * @param x1 The x coordinate of the first point.
 * @param y1 The y coordinate of the first point.
 * @param x2 The x coordinate of the second point.
 * @param y2 The y coordinate of the second point.
 * @return The angle in radians from (x1, y1) to (x2, y2).
 */
inline double angle(
  const double x1,
  const double y1,
  const double x2,
  const double y2)
{
  return std::atan2(y2 - y1, x2 - x1);
}

/**
 * @brief Compute the shortest signed angular distance between two angles.
 * Wraps the result into [-pi, pi].
 *
 * @param from Starting angle in radians.
 * @param to Ending angle in radians.
 * @return The minimal signed angular difference in radians.
 */
inline double shortest_angular_distance(
  const double from,
  const double to)
{
  return angles::shortest_angular_distance(from, to);
}

}  // namespace nav2_drone_util
