// include/nav2_drone_planner/planner_server.hpp

#ifndef NAV2_DRONE_PLANNER__PLANNER_SERVER_HPP_
#define NAV2_DRONE_PLANNER__PLANNER_SERVER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "nav2_drone_core/planner.hpp"
#include "nav2_drone_msgs/action/compute_path_to_pose.hpp"

namespace nav2_drone_planner
{

class PlannerServer : public rclcpp::Node
{
public:
  using ComputePath = nav2_drone_msgs::action::ComputePathToPose;
  using GoalHandleComputePath = rclcpp_action::ServerGoalHandle<ComputePath>;

  explicit PlannerServer(const rclcpp::NodeOptions & options);

private:
  // Action callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePath::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleComputePath> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleComputePath> goal_handle);

  void execute(
    const std::shared_ptr<GoalHandleComputePath> goal_handle);

  // Members
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  pluginlib::ClassLoader<nav2_drone_core::Planner> loader_;
  std::map<std::string, nav2_drone_core::Planner::Ptr> planners_;
  std::vector<std::string> default_ids_, default_types_;
  rclcpp_action::Server<ComputePath>::SharedPtr action_server_;

  // Additional parameters and state
  std::string map_frame_, robot_base_frame_;
  double transform_tolerance_;
};

}  // namespace nav2_drone_planner

#endif  // NAV2_DRONE_PLANNER__PLANNER_SERVER_HPP_
