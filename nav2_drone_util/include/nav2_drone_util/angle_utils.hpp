#pragma once

#include <cmath>
#include <angles/angles.h>

namespace nav2_drone_util {

/**
 * @brief Calculate the angle between a line defined by two points and the coordinate axes.
 * Result is in radians.
 *
 * @param x1 The x coordinate of the first point.
 * @param y1 The y coordinate of the first point.
 * @param x2 The x coordinate of the second point.
 * @param y2 The y coordinate of the second point.
 * @return The angle (in radians) from (x1, y1) to (x2, y2), measured relative to the x-axis.
 */
double angle(
  const double x1,
  const double y1,
  const double x2,
  const double y2);

/**
 * @brief Compute the shortest angular distance between two angles (radians).
 * Wraps around at 2π to provide the minimal signed angle difference.
 *
 * @param from Starting angle (radians).
 * @param to Ending angle (radians).
 * @return The minimal signed angular difference (radians) in [-π, π].
 */
inline double shortest_angular_distance(
  const double from,
  const double to)
{
  return angles::shortest_angular_distance(from, to);
}

}  // namespace nav2_drone_util
