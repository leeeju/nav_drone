#ifndef NAV2_DRONE_UTIL__GEOMETRY_UTILS_HPP_
#define NAV2_DRONE_UTIL__GEOMETRY_UTILS_HPP_

#include <cmath>
#include <stdexcept>
#include <string>
#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_drone_util
{

// Exception for geometry-related errors
class GeometryException : public std::runtime_error
{
public:
  explicit GeometryException(const std::string & description)
  : std::runtime_error(description) {}
};

// Exception for boundary-related geometry errors
class GeometryBoundsError : public GeometryException
{
public:
  explicit GeometryBoundsError(const std::string & description)
  : GeometryException(description) {}
};

/**
 * @brief Converts yaw angle to a geometry_msgs Quaternion
 * @param angle Yaw angle in radians
 * @return geometry_msgs Quaternion
 */
inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // Only rotate around the Z-axis
  return tf2::toMsg(q);
}

/**
 * @brief Computes the Euclidean distance between two geometry_msgs Points
 * @param pos1 First point
 * @param pos2 Second point
 * @param is_3d Calculate 3D distance if true, otherwise 2D distance
 * @return double Euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2,
  const bool is_3d = false)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  if (is_3d) {
    double dz = pos1.z - pos2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return std::hypot(dx, dy);
}

/**
 * @brief Computes the Euclidean distance between two geometry_msgs Poses
 * @param pos1 First pose
 * @param pos2 Second pose
 * @param is_3d Calculate 3D distance if true, otherwise 2D distance
 * @return double Euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Pose & pos1,
  const geometry_msgs::msg::Pose & pos2,
  const bool is_3d = false)
{
  return euclidean_distance(pos1.position, pos2.position, is_3d);
}

/**
 * @brief Computes the Euclidean distance between two geometry_msgs PoseStamped
 * @param pos1 First pose
 * @param pos2 Second pose
 * @param is_3d Calculate 3D distance if true, otherwise 2D distance
 * @return double Euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::PoseStamped & pos1,
  const geometry_msgs::msg::PoseStamped & pos2,
  const bool is_3d = false)
{
  return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}

/**
 * @brief Computes the total length of a given path
 * @param path Path to measure
 * @param start_index Starting index for measurement
 * @return double Path length
 */
inline double calculate_path_length(const nav_msgs::msg::Path & path, size_t start_index = 0)
{
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}

/**
 * @brief Finds the intersection of a sphere and a line segment
 * @param p1 First endpoint of the segment
 * @param p2 Second endpoint of the segment
 * @param cen Center of the sphere
 * @param r Radius of the sphere
 * @return geometry_msgs::msg::Point Intersection point
 * @throws GeometryBoundsError if no intersection exists
 */
inline geometry_msgs::msg::Point sphereSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & cen,
  double r)
{
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double dz = p2.z - p1.z;

  double a = dx * dx + dy * dy + dz * dz;
  double b = 2.0 * ((p1.x - cen.x) * dx + (p1.y - cen.y) * dy + (p1.z - cen.z) * dz);
  double c = (p1.x - cen.x) * (p1.x - cen.x) +
             (p1.y - cen.y) * (p1.y - cen.y) +
             (p1.z - cen.z) * (p1.z - cen.z) - r * r;

  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    throw GeometryBoundsError("Line segment does not intersect sphere");
  }

  double t1 = (-b - std::sqrt(discriminant)) / (2.0 * a);
  double t2 = (-b + std::sqrt(discriminant)) / (2.0 * a);

  geometry_msgs::msg::Point intersection1;
  intersection1.x = p1.x + t1 * dx;
  intersection1.y = p1.y + t1 * dy;
  intersection1.z = p1.z + t1 * dz;

  geometry_msgs::msg::Point intersection2;
  intersection2.x = p1.x + t2 * dx;
  intersection2.y = p1.y + t2 * dy;
  intersection2.z = p1.z + t2 * dz;

  // Return the intersection point closest to p1 within the segment
  if (0.0 <= t1 && t1 <= 1.0) {
    return intersection1;
  } else if (0.0 <= t2 && t2 <= 1.0) {
    return intersection2;
  }

  throw GeometryBoundsError("Intersection points lie outside the segment");
}

}  // namespace nav2_drone_util

#endif  // nav2_drone_util__GEOMETRY_UTILS_HPP_
