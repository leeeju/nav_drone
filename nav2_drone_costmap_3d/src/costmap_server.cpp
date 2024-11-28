#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_drone_costmap_3d/costmap_server.hpp"
#include "nav2_drone_util/robot_utils.hpp"
#include "nav2_drone_util/geometry_utils.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

using namespace std::chrono_literals;

namespace nav2_drone_costmap_3d {

CostmapPublisher::CostmapPublisher(const std::string &name)
: Node(name), current_(false), octomap_(nullptr), last_octomap_update_(rclcpp::Time(0, 0, RCL_ROS_TIME))
{
  rclcpp::init();
}

void CostmapPublisher::rclcpp::init()
{
  // Declare and get parameters
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame", "base_link");
  transform_tolerance_ = this->declare_parameter<double>("transform_tolerance", 0.1);
  lookahead_dist_ = this->declare_parameter<double>("lookahead_dist", 2.0);
  robot_radius_ = this->declare_parameter<double>("robot_radius", 0.5);
  safety_radius_ = this->declare_parameter<double>("safety_radius", 0.3);

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize subscriptions
  map_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "nav_drone/map", 10,
    std::bind(&CostmapPublisher::map_callback, this, std::placeholders::_1));

  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "drone/odom", 10,
    std::bind(&CostmapPublisher::odom_callback, this, std::placeholders::_1));

  // Initialize publisher
  costmap_publisher_ = this->create_publisher<nav2_drone_msgs::msg::Costmap>("nav_drone/costmap", 10);

  // Timer for periodic costmap publishing
  timer_ = this->create_wall_timer(
    500ms, std::bind(&CostmapPublisher::publish_costmap, this));
}

void CostmapPublisher::map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex);
  octomap_ = std::make_shared<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::fullMsgToMap(*msg)));
  last_octomap_update_ = this->get_clock()->now();
  RCLCPP_INFO(this->get_logger(), "Octomap updated successfully.");
}

void CostmapPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_velocity_ = msg->twist.twist;
}

bool CostmapPublisher::getRobotPose(geometry_msgs::msg::PoseStamped &global_pose)
{
  return nav_drone_util::getCurrentPose(
    global_pose, *tf_buffer_, map_frame_, robot_base_frame_, transform_tolerance_);
}

void CostmapPublisher::publish_costmap()
{
  if (!octomap_) {
    RCLCPP_WARN(this->get_logger(), "No Octomap available, skipping costmap publish.");
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!getRobotPose(current_pose)) {
    RCLCPP_WARN(this->get_logger(), "Failed to get robot pose.");
    return;
  }

  auto message = nav2_drone_msgs::msg::Costmap();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = map_frame_;

  {
    std::lock_guard<std::mutex> lock(costmap_mutex);
    message.data.resize(costmap_->z_dim() * costmap_->e_dim());
  }

  costmap_publisher_->publish(message);
}

}  // namespace nav_drone_costmap_3d
