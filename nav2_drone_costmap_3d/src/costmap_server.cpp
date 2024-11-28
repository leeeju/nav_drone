#include "nav2_drone_costmap_3d/costmap_server.hpp"
#include "nav2_drone_costmap_3d/costmap_3d.hpp"
#include "nav2_drone_util/robot_utils.hpp"
#include "nav2_drone_util/node_utils.hpp"
// #include "nav2_drone_core/costmap_exceptions.hpp"

#include <octomap_msgs/conversions.h>

namespace nav2_drone_costmap_3d {

CostmapPublisher::CostmapPublisher(const std::string &name)
    : Node(name),
      current_(false),
      costmap_(std::make_shared<Costmap3D>(ALPHA_RES)),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      transform_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
  init();
}

void CostmapPublisher::init() {
  // Declare parameters
  nav2_drone_util::declare_parameter_if_not_declared(this, "map_frame", rclcpp::ParameterValue("map"));
  nav2_drone_util::declare_parameter_if_not_declared(this, "robot_base_frame", rclcpp::ParameterValue("base_link"));
  nav2_drone_util::declare_parameter_if_not_declared(this, "transform_tolerance", rclcpp::ParameterValue(0.3));
  nav2_drone_util::declare_parameter_if_not_declared(this, "lookahead_dist", rclcpp::ParameterValue(2.0));
  nav2_drone_util::declare_parameter_if_not_declared(this, "robot_radius", rclcpp::ParameterValue(0.5));
  nav2_drone_util::declare_parameter_if_not_declared(this, "safety_radius", rclcpp::ParameterValue(0.5));

  // Get parameters
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("transform_tolerance", transform_tolerance_);
  this->get_parameter("lookahead_dist", lookahead_dist_);
  this->get_parameter("robot_radius", robot_radius_);
  this->get_parameter("safety_radius", safety_radius_);

  // Ensure TF2 transform is available
  std::string tf_error;
  while (rclcpp::ok() && !tf_buffer_->canTransform(map_frame_, robot_base_frame_, tf2::TimePointZero, &tf_error)) {
    RCLCPP_INFO(get_logger(), "Waiting for transform from %s to %s: %s",
                robot_base_frame_.c_str(), map_frame_.c_str(), tf_error.c_str());
    tf_error.clear();
    std::this_thread::sleep_for(100ms);
  }

  // Subscriptions
  map_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "nav_drone/map", 10, std::bind(&CostmapPublisher::map_callback, this, std::placeholders::_1));
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "drone/odom", 10, std::bind(&CostmapPublisher::odom_callback, this, std::placeholders::_1));

  // Publisher
  costmap_publisher_ = this->create_publisher<nav2_drone_msgs::msg::Costmap>("nav_drone/costmap", 10);

  // Timer
  timer_ = this->create_wall_timer(500ms, std::bind(&CostmapPublisher::publish_costmap, this));
}

void CostmapPublisher::map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
  if (tree) {
    octomap_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(tree));
    last_octomap_update_ = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Octomap updated successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message.");
  }
}

void CostmapPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  last_velocity_ = msg->twist.twist;
}

bool CostmapPublisher::update_costmap(const geometry_msgs::msg::PoseStamped &current_pose, double bounding_box_radius) {
  if (!octomap_) {
    throw nav2_drone_core::CostmapUpdateError("No Octomap available for costmap update.");
  }

  costmap_mutex.lock();
  costmap_->set_zero();
  costmap_mutex.unlock();

  octomap::point3d min(current_pose.pose.position.x - bounding_box_radius,
                       current_pose.pose.position.y - bounding_box_radius,
                       current_pose.pose.position.z - bounding_box_radius);
  octomap::point3d max(current_pose.pose.position.x + bounding_box_radius,
                       current_pose.pose.position.y + bounding_box_radius,
                       current_pose.pose.position.z + bounding_box_radius);

  for (auto it = octomap_->begin_leafs_bbx(min, max), end = octomap_->end_leafs_bbx(); it != end; ++it) {
    if (octomap_->isNodeOccupied(*it)) {
      auto coords = get_ez_grid_pos(it.getCoordinate());
      costmap_mutex.lock();
      costmap_->set_weight(coords.first, coords.second, 1.0);  // Example: Mark as occupied
      costmap_mutex.unlock();
    }
  }

  return true;
}

std::pair<int, int> CostmapPublisher::get_ez_grid_pos(const octomap::point3d &point) {
  int elevation_idx = static_cast<int>((point.z() / octomap_->getResolution()) + 0.5);
  int azimuth_idx = static_cast<int>((atan2(point.y(), point.x()) * 180.0 / M_PI) / ALPHA_RES + 0.5);
  return {elevation_idx, azimuth_idx};
}

void CostmapPublisher::publish_costmap() {
  if (!octomap_) {
    RCLCPP_WARN(this->get_logger(), "No Octomap available, skipping costmap publish.");
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_drone_util::getCurrentPose(current_pose, *tf_buffer_, map_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to get robot pose.");
    return;
  }

  try {
    double bounding_box_radius = lookahead_dist_;
    update_costmap(current_pose, bounding_box_radius);
  } catch (const nav2_drone_core::CostmapUpdateError &e) {
    RCLCPP_ERROR(this->get_logger(), "Costmap update failed: %s", e.what());
    return;
  }

  auto message = nav2_drone_msgs::msg::Costmap();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = map_frame_;

  costmap_mutex.lock();
  message.data.resize(costmap_->z_dim() * costmap_->e_dim());
  for (int e = 0; e < costmap_->e_dim(); ++e) {
    for (int z = 0; z < costmap_->z_dim(); ++z) {
      message.data[e * costmap_->z_dim() + z] = costmap_->get_weight(e, z) > 0.0 ? 100 : 0;  // Example: Binary cost
    }
  }
  costmap_mutex.unlock();

  costmap_publisher_->publish(message);
}

bool CostmapPublisher::getRobotPose(geometry_msgs::msg::PoseStamped &global_pose) {
  return nav2_drone_util::getCurrentPose(global_pose, *tf_buffer_, map_frame_, robot_base_frame_, transform_tolerance_);
}

}  // namespace nav2_drone_costmap_3d
