#include "nav2_drone_util/tf_help.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>

namespace nav2_drone_util
{

bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  rclcpp::Duration & transform_tolerance)
{
  // If the input pose is already in the desired frame, no transformation is needed.
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    // Attempt to transform the pose using the TF buffer
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (const tf2::ExtrapolationException & ex) {
    // Handle extrapolation exceptions by attempting a fallback transformation
    try {
      auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);

      // Check if the transform is too old based on tolerance
      rclcpp::Time data_time(in_pose.header.stamp);
      rclcpp::Time transform_time(transform.header.stamp);

      if ((data_time - transform_time) > transform_tolerance) {
        RCLCPP_ERROR(
          rclcpp::get_logger("nav2_drone_util.tf_help"),
          "Transform data too old when converting from %s to %s. Data time: %ds %uns, "
          "Transform time: %ds %uns, Transform tolerance: %.2fs",
          in_pose.header.frame_id.c_str(), frame.c_str(),
          data_time.seconds(), data_time.nanoseconds(),
          transform_time.seconds(), transform_time.nanoseconds(),
          transform_tolerance.seconds());
        return false;
      }

      // Perform the transformation using the fallback
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    } catch (const tf2::TransformException & ex2) {
      RCLCPP_ERROR(
        rclcpp::get_logger("nav2_drone_util.tf_help"),
        "Failed fallback transformation: %s", ex2.what());
      return false;
    }
  } catch (const tf2::TransformException & ex) {
    // Log any other TF exceptions
    RCLCPP_ERROR(
      rclcpp::get_logger("nav2_drone_util.tf_help"),
      "Exception during transformPose: %s", ex.what());
    return false;
  }

  return false;
}

}  // namespace nav2_drone_util
