#ifndef NAV2_DRONE_UTIL__ODOMETRY_UTILS_HPP_
#define NAV2_DRONE_UTIL__ODOMETRY_UTILS_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_drone_util
{

class OdometryUtils
{
public:
  OdometryUtils() = default;
  explicit OdometryUtils(std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /**
   * @brief Compute drone speed from odometry message considering XYZ axes.
   */
  double getLinearSpeed(const nav_msgs::msg::Odometry &odom) const;

  /**
   * @brief Compute drone angular speed from odometry message.
   */
  double getAngularSpeed(const nav_msgs::msg::Odometry &odom) const;

  /**
   * @brief Get pose transformed into global reference frame considering drone altitude.
   */
  bool getTransformedPose(
    const geometry_msgs::msg::Pose &input_pose,
    geometry_msgs::msg::Pose &output_pose,
    const std::string &from_frame,
    const std::string &to_frame,
    double transform_tolerance) const;

  /**
   * @brief Convert quaternion to roll, pitch, yaw angles for drone orientation.
   */
  void quaternionToRPY(
    const geometry_msgs::msg::Quaternion &q,
    double &roll, double &pitch, double &yaw) const;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace nav2_drone_util

#endif  // NAV2_DRONE_UTIL__ODOMETRY_UTILS_HPP_
