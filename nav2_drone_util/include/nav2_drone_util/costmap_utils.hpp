#pragma once

#include <utility>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_drone_util {

/**
 * @brief Calculate the elevation (plunge) and azimuth angles between two poses in 3D space.
 *
 * This function computes the angular relationship (elevation and azimuth) between the current pose
 * and a target pose. Elevation (plunge) is the vertical angle (positive for upward), while azimuth
 * is the horizontal bearing.
 *
 * @param current_pose The current pose of the drone in 3D space.
 * @param target_pose The target pose of the drone in 3D space.
 * @return A pair of doubles: (plunge, azimuth) in radians.
 */
std::pair<double, double> calculate_ez(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose);

}  // namespace nav2_drone_util
