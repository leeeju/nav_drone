#ifndef NAV2_DRONE_CONTROLLER__CONTROLLER_SERVER_HPP_
#define NAV2_DRONE_CONTROLLER__CONTROLLER_SERVER_HPP_

#include "nav2_drone_costmap_3d/costmap_publisher.hpp"
#include "nav2_drone_costmap_3d/layered_costmap_3d.hpp"
#include "nav2_drone_util/path_utils.hpp"
#include "nav2_drone_util/tf_help.hpp"
#include <pluginlib/class_loader.hpp>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace nav2_drone_controller
{

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: Node("controller_server", options),
  progress_checker_loader_("nav2_drone_core", "nav2_drone_core::ProgressChecker"),
  goal_checker_loader_("nav2_drone_core", "nav2_drone_core::GoalChecker"),
  lp_loader_("nav2_drone_core", "nav2_drone_core::Controller")
{
  init();
}

ControllerServer::~ControllerServer() = default;

void ControllerServer::init()
{
  // Declare and get resolution parameter
  declare_parameter_if_not_declared(shared_from_this(), "resolution", rclcpp::ParameterValue(5));
  int resolution = get_parameter("resolution").as_int();

  // Create layered costmap
  layered_map_ = std::make_shared<LayeredCostmap3D>(resolution);

  // Start costmap publisher node in its own thread
  costmap_publisher_node_ = std::make_shared<CostmapPublisher>(rclcpp::NodeOptions());
  costmap_thread_ = std::make_unique<nav2_drone_util::NodeThread>(costmap_publisher_node_);

  // Velocity publisher
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Odometry subscription
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&ControllerServer::odom_callback, this, std::placeholders::_1));

  // Action server
  action_server_ = rclcpp_action::create_server<FollowPath>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "follow_path",
    std::bind(&ControllerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ControllerServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ControllerServer::handle_accepted, this, std::placeholders::_1));

  // Load controller plugins
  node_thread_comments(); // placeholder for loading LP, GC, PC plugins
}

rclcpp_action::GoalResponse ControllerServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowPath::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Received FollowPath goal request");
  // Always accept
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ControllerServer::handle_cancel(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ControllerServer::handle_accepted(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  std::thread(&ControllerServer::execute, this, goal_handle).detach();
}

void ControllerServer::execute(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing goal");
  auto goal = goal_handle->get_goal();
  // Get path
  auto path = goal->path;
  end_pose_ = path.back();

  rclcpp::Rate rate(controller_frequency_);
  while (rclcpp::ok()) {
    geometry_msgs::msg::PoseStamped current_pose;
    if (!getRobotPose(current_pose)) {
      RCLCPP_WARN(get_logger(), "Could not get robot pose");
      continue;
    }

    if (isGoalReached()) {
      publishZeroVelocity();
      goal_handle->succeed(result_);
      return;
    }

    // Compute velocity command
    auto twist = controllers_[current_controller_]->computeVelocityCommand(
      path, current_pose, layered_map_);
    auto twist_thresh = getThresholdedTwist(twist);
    publisher_->publish(twist_thresh);

    rate.sleep();
  }
}

bool ControllerServer::publishZeroVelocity()
{
  auto zero = geometry_msgs::msg::Twist();
  publisher_->publish(zero);
  return true;
}

bool ControllerServer::getRobotPose(
  geometry_msgs::msg::PoseStamped & pose)
{
  return nav2_drone_util::getRobotPose(pose,
    costmap_publisher_node_->get_node_base_interface(),
    "map");
}

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped current_pose;
  getRobotPose(current_pose);
  auto gc = goal_checkers_[current_goal_checker_];
  return gc->isGoalReached(current_pose, end_pose_, last_velocity_.twist);
}

void ControllerServer::odom_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_velocity_.twist = msg->twist.twist;
}

bool ControllerServer::findControllerId(
  const std::string & c_name,
  std::string & name)
{
  // Implementation stub
  name = default_ids_.front();
  return true;
}

bool ControllerServer::findGoalCheckerId(
  const std::string & c_name,
  std::string & name)
{
  // Implementation stub
  name = default_goal_checker_ids_.front();
  return true;
}

}  // namespace nav2_drone_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_drone_controller::ControllerServer,
  rclcpp::Node)
