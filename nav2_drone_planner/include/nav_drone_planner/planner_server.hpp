#ifndef NAV2_DRONE_PLANNER__PLANNER_SERVER_HPP_
#define NAV2_DRONE_PLANNER__PLANNER_SERVER_HPP_

#include <unordered_map>
#include <iterator>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

// Updated to use nav2_drone_msgs
#include "nav2_drone_msgs/action/compute_path_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>

// Updated utility and core includes
#include "nav2_drone_util/drone_utils.hpp"
#include "nav2_drone_util/node_utils.hpp"
#include "nav2_drone_util/visibility_control.h"
#include <nav2_drone_core/planner.hpp>

#include <pluginlib/class_loader.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

namespace nav_drone_planner
{

class PlannerServer : public rclcpp::Node
{
public:
  using ComputePathAction = nav2_drone_msgs::action::ComputePathToPose;
  using GoalHandleComputePath = rclcpp_action::ServerGoalHandle<ComputePathAction>;

  NAV_DRONE_UTIL_PUBLIC explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  // Node parameters
  std::string map_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;

  // Planner plugin loader
  using PlannerMap = std::unordered_map<std::string, std::shared_ptr<nav2_drone_core::Planner>>;
  PlannerMap planners_;
  pluginlib::ClassLoader<nav2_drone_core::Planner> loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;

  // Action server
  rclcpp_action::Server<ComputePathAction>::SharedPtr action_server_;

  // Utilities
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Member functions
  void goal_callback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathAction::Goal> goal_request);

  void cancel_callback(const std::shared_ptr<GoalHandleComputePath> goal_handle);

  void accepted_callback(const std::shared_ptr<GoalHandleComputePath> goal_handle);

  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id);

  bool validate_path(
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::Path & path,
    const std::string & planner_id);
};  // class PlannerServer

}  // namespace nav_drone_planner

#endif  // NAV_DRONE_PLANNER__PLANNER_SERVER_HPP_
