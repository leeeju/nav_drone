#include "nav2_drone_controller/plugins/simple_progress_checker.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_drone_util/node_utils.hpp"
#include "nav2_drone_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using std::placeholders::_1;

namespace nav2_drone_controller
{

void SimpleProgressChecker::initialize(
  const rclcpp::Node::SharedPtr parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent;

  clock_ = node->get_clock();

  rcl_interfaces::msg::ParameterDescriptor desc;
  nav2_drone_util::declare_parameter_if_not_declared(
    node, plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5), desc);
  nav2_drone_util::declare_parameter_if_not_declared(
    node, plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0), desc);
  // Scale is set to 0 by default, so if it was not set otherwise, set to 0
  node->get_parameter_or(plugin_name + ".required_movement_radius", radius_, 0.5);
  double time_allowance_param = 0.0;
  node->get_parameter_or(plugin_name + ".movement_time_allowance", time_allowance_param, 10.0);
  time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);

}

bool SimpleProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // baseline_pose is not set.
  if ((!baseline_pose_set_) || (is_robot_moved_enough(current_pose.pose))) {
    reset_baseline_pose(current_pose.pose);
    return true;
  }
  return !((clock_->now() - baseline_time_) > time_allowance_);
}

void SimpleProgressChecker::reset()
{
  baseline_pose_set_ = false;
}

void SimpleProgressChecker::reset_baseline_pose(const geometry_msgs::msg::Pose & pose)
{
  baseline_pose_ = pose;
  baseline_time_ = clock_->now();
  baseline_pose_set_ = true;
}

bool SimpleProgressChecker::is_robot_moved_enough(const geometry_msgs::msg::Pose & pose)
{
  return nav2_drone_util::euclidean_distance(pose, baseline_pose_) > radius_;
}


}  // namespace nav2_drone_controller

PLUGINLIB_EXPORT_CLASS(nav2_drone_controller::SimpleProgressChecker, nav2_drone_core::ProgressChecker)
