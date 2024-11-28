#ifndef NAV_DRONE_UTIL__GEOMETRY_UTILS_HPP_
#define NAV_DRONE_UTIL__GEOMETRY_UTILS_HPP_

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_drone_util
{

/**
 * @brief Exception class for geometry-related errors
 */
class GeometryException : public std::runtime_error
{
public:
  explicit GeometryException(const std::string & description)
  : std::runtime_error(description) {}
};

/**
 * @brief Exception for errors related to geometry bounds
 */
class GeometryBoundsError : public GeometryException
{
public:
  explicit GeometryBoundsError(const std::string & description)
  : GeometryException(description) {}
};

/**
 * @brief Get a quaternion representing rotation about the Z-axis
 * @param angle Yaw angle in radians
 * @return Quaternion representing the rotation
 */
inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // Rotation about Z-axis
  return tf2::toMsg(q);
}

/**
 * @brief Compute the Euclidean distance between two points
 * @param pos1 First point
 * @param pos2 Second point
 * @param is_3d If true, computes the 3D distance; otherwise, computes the 2D distance (default false)
 * @return Euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2,
  bool is_3d = false)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  if (is_3d) {
    double dz = pos1.z - pos2.z;
    return std::hypot(dx, std::hypot(dy, dz));
  }

  return std::hypot(dx, dy);
}

/**
 * @brief Compute the Euclidean distance between two poses
 * @param pose1 First pose
 * @param pose2 Second pose
 * @param is_3d If true, computes the 3D distance; otherwise, computes the 2D distance (default false)
 * @return Euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Pose & pose1,
  const geometry_msgs::msg::Pose & pose2,
  bool is_3d = false)
{
  return euclidean_distance(pose1.position, pose2.position, is_3d);
}

/**
 * @brief Compute the Euclidean distance between two stamped poses
 * @param pose1 First pose
 * @param pose2 Second pose
 * @param is_3d If true, computes the 3D distance; otherwise, computes the 2D distance (default false)
 * @return Euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2,
  bool is_3d = false)
{
  return euclidean_distance(pose1.pose, pose2.pose, is_3d);
}

/**
 * @brief Calculate the total length of a path
 * @param path The path consisting of a sequence of poses
 * @param start_index The index to start calculating from (default 0)
 * @return Total path length
 */
inline double calculate_path_length(const nav_msgs::msg::Path & path, size_t start_index = 0)
{
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }

  double path_length = 0.0;
  for (size_t i = start_index; i < path.poses.size() - 1; ++i) {
    path_length += euclidean_distance(path.poses[i].pose, path.poses[i + 1].pose);
  }

  return path_length;
}

/**
 * @brief Find the intersection point of a line segment and a sphere
 * @param p1 Start point of the segment
 * @param p2 End point of the segment
 * @param center Center of the sphere
 * @param radius Radius of the sphere
 * @return Intersection point on the segment
 * @throws GeometryBoundsError if the segment does not intersect the sphere
 */
inline geometry_msgs::msg::Point sphereSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & center,
  double radius)
{
  // Calculate coefficients of the quadratic equation
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double dz = p2.z - p1.z;

  double a = dx * dx + dy * dy + dz * dz;
  double b = 2 * (dx * (p1.x - center.x) + dy * (p1.y - center.y) + dz * (p1.z - center.z));
  double c = (p1.x - center.x) * (p1.x - center.x) +
             (p1.y - center.y) * (p1.y - center.y) +
             (p1.z - center.z) * (p1.z - center.z) -
             radius * radius;

  double discriminant = b * b - 4 * a * c;

  if (discriminant < 0) {
    throw GeometryBoundsError("Line segment does not intersect sphere");
  }

  double t1 = (-b - std::sqrt(discriminant)) / (2 * a);
  double t2 = (-b + std::sqrt(discriminant)) / (2 * a);

  // Ensure t1 and t2 are within the segment range [0, 1]
  double t = (t1 >= 0 && t1 <= 1) ? t1 : t2;
  if (t < 0 || t > 1) {
    throw GeometryBoundsError("Intersection point is outside the segment");
  }

  // Calculate intersection point
  geometry_msgs::msg::Point intersection;
  intersection.x = p1.x + t * dx;
  intersection.y = p1.y + t * dy;
  intersection.z = p1.z + t * dz;

  return intersection;
}

}  // namespace nav_drone_util

#endif  // NAV_DRONE_UTIL__GEOMETRY_UTILS_HPP_
