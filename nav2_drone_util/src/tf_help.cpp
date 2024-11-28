#include <memory>
#include <string>
#include "nav2_drone_util/tf_help.hpp"

namespace nav2_drone_util
{

bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance)
{
  static const rclcpp::Logger logger = rclcpp::get_logger("nav_drone_util.tf_help");

  // If the pose is already in the target frame, return it directly
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    // Attempt to transform using the TF buffer
    out_pose = tf->transform(in_pose, frame, tf2::durationFromSec(transform_tolerance.seconds()));
    return true;
  } catch (const tf2::ExtrapolationException & ex) {
    // Handle extrapolation errors by inspecting timestamps
    RCLCPP_WARN(
      logger, "ExtrapolationException: %s. Trying fallback transformation.", ex.what());

    try {
      // Attempt to lookup the transform directly
      auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);

      rclcpp::Time data_time(in_pose.header.stamp);
      rclcpp::Time transform_time(transform.header.stamp);

      // Check if the transform data is too old
      if ((data_time - transform_time) > transform_tolerance) {
        RCLCPP_ERROR(
          logger,
          "Transform data too old: from %s to %s. Data time: %ds %uns, "
          "Transform time: %ds %uns, Transform tolerance: %.2fs",
          in_pose.header.frame_id.c_str(), frame.c_str(),
          data_time.seconds(), data_time.nanoseconds(),
          transform_time.seconds(), transform_time.nanoseconds(),
          transform_tolerance.seconds());
        return false;
      }

      // Perform transformation using the fallback transform
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    } catch (const tf2::TransformException & ex2) {
      RCLCPP_ERROR(logger, "Failed fallback transformation: %s", ex2.what());
      return false;
    }
  } catch (const tf2::TransformException & ex) {
    // Handle all other TF transform exceptions
    RCLCPP_ERROR(logger, "TransformException: %s", ex.what());
    return false;
  }
}

}  // namespace nav2_drone_util
