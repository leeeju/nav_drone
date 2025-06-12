#pragma once

#include <string>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace nav2_drone_msgs
{
namespace action
{

// Goal definition
struct ComputePathToPose_Goal
{
  geometry_msgs::msg::PoseStamped start;
  geometry_msgs::msg::PoseStamped goal;
  std::string planner_id;
  builtin_interfaces::msg::Duration max_planning_time;
};

// Result definition
struct ComputePathToPose_Result
{
  nav_msgs::msg::Path path;
  bool success;
  std::string error_message;
};

// Feedback definition
struct ComputePathToPose_Feedback
{
  nav_msgs::msg::Path partial_path;
  float progress;
};

// Action definition
struct ComputePathToPose
{
  using Goal = ComputePathToPose_Goal;
  using Result = ComputePathToPose_Result;
  using Feedback = ComputePathToPose_Feedback;
};

}  // namespace action
}  // namespace nav2_drone_msgs
