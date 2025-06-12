#include "nav2_drone_planner/planner_server.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace nav2_drone_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: Node("nav2_drone_planner", options)
, loader_("nav2_drone_core", "nav2_drone_core::Planner")
{
  // Parameters
  map_frame_ = this->declare_parameter("map_frame", std::string("map"));
  robot_base_frame_ = this->declare_parameter("base_frame", std::string("base_link"));
  transform_tolerance_ = this->declare_parameter("transform_tolerance", 0.1);

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Plugin loader initial values
  default_ids_   = {"DumbPlanner", "ThetaStarPlanner"};
  default_types_ = {"nav_drone_dumb_planner/DumbPlanner",
                    "nav_drone_theta_star_planner/ThetaStarPlanner"};

  for (size_t i = 0; i < default_ids_.size(); ++i) {
    try {
      auto planner = loader_.createSharedInstance(default_types_[i]);
      planners_.emplace(default_ids_[i], planner);
      RCLCPP_INFO(this->get_logger(),
                  "Loaded planner %s of type %s",
                  default_ids_[i].c_str(), default_types_[i].c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to load %s: %s",
                  default_types_[i].c_str(), e.what());
    }
  }

  // Action server
  using ComputePath = nav2_drone_msgs::action::ComputePathToPose;
  action_server_ = rclcpp_action::create_server<ComputePath>(
    this,
    "compute_path_to_pose",
    std::bind(&PlannerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PlannerServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&PlannerServer::handle_accepted, this, std::placeholders::_1)
  );
}

rclcpp_action::GoalResponse PlannerServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PlannerServer::ComputePath::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(),
              "Received goal to (%.2f, %.2f, %.2f)",
              goal->pose.pose.position.x,
              goal->pose.pose.position.y,
              goal->pose.pose.position.z);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerServer::handle_cancel(
  const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Cancel request received");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerServer::handle_accepted(
  const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
  std::thread{std::bind(&PlannerServer::execute, this, goal_handle)}.detach();
}

void PlannerServer::execute(
  const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
  auto result = std::make_shared<ComputePath::Result>();
  // TODO: Implement planning logic
  goal_handle->succeed(result);
}

}  // namespace nav2_drone_planner

RCLCPP_COMPONENTS_REGISTER_NODE(nav2_drone_planner::PlannerServer)
