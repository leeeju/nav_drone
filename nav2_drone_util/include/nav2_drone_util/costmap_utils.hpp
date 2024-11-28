#pragma once

#include <utility>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_drone_util {

/**
 * @brief Calculate elevation and azimuth angles between the current pose and the target pose.
 * @param current_pose Current position and orientation of the robot (PoseStamped).
 * @param target_pose Target position and orientation of the robot (PoseStamped).
 * @return A pair of doubles representing the elevation (E) and azimuth (Z) angles in radians.
 */
inline std::pair<double, double> calculate_ez(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const geometry_msgs::msg::PoseStamped &target_pose)
{
    // Extract the position components from the poses
    const auto &current_position = current_pose.pose.position;
    const auto &target_position = target_pose.pose.position;

    // Calculate differences in x, y, and z
    double dx = target_position.x - current_position.x;
    double dy = target_position.y - current_position.y;
    double dz = target_position.z - current_position.z;

    // Compute the azimuth (Z) and elevation (E) angles
    double azimuth = std::atan2(dy, dx); // Azimuth: angle in the XY plane
    double distance = std::sqrt(dx * dx + dy * dy); // Distance in the XY plane
    double elevation = std::atan2(dz, distance); // Elevation: angle above the XY plane

    return std::make_pair(elevation, azimuth);
}

/**
 * @brief Compute the 3D distance between two poses.
 * @param pose1 First pose (PoseStamped).
 * @param pose2 Second pose (PoseStamped).
 * @return Euclidean distance between the two poses in meters.
 */
inline double compute_3d_distance(
    const geometry_msgs::msg::PoseStamped &pose1,
    const geometry_msgs::msg::PoseStamped &pose2)
{
    const auto &pos1 = pose1.pose.position;
    const auto &pos2 = pose2.pose.position;

    return std::sqrt(
        std::pow(pos2.x - pos1.x, 2) +
        std::pow(pos2.y - pos1.y, 2) +
        std::pow(pos2.z - pos1.z, 2));
}

/**
 * @brief Normalize an angle to the range [-PI, PI).
 * @param angle Angle in radians.
 * @return Normalized angle in radians.
 */
inline double normalize_angle(double angle) {
    const double PI = 3.141592653589793238463;
    const double TWO_PI = 2.0 * PI;

    double result = fmod(angle + PI, TWO_PI);
    return result < 0.0 ? result + PI : result - PI;
}

} // namespace nav2_drone_util
