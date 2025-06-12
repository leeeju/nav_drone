#ifndef NAV2_DRONE_MSGS__MSG__ODOMETRY_HPP_
#define NAV2_DRONE_MSGS__MSG__ODOMETRY_HPP_

#include <string>
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"

namespace nav2_drone_msgs
{
namespace msg
{

/// \brief Drone-specific odometry message
struct Odometry
{
  using SharedPtr = std::shared_ptr<Odometry>;

  /// Timestamped header
  std_msgs::msg::Header header;

  /// Child frame ID (e.g., base_link)
  std::string child_frame_id;

  /// Pose of the robot/drone with covariance
  geometry_msgs::msg::PoseWithCovariance pose;

  /// Twist of the robot/drone with covariance
  geometry_msgs::msg::TwistWithCovariance twist;
};

}  // namespace msg
}  // namespace nav2_drone_msgs

#endif  // NAV2_DRONE_MSGS__MSG__ODOMETRY_HPP_
