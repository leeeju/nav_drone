#include "nav2_drone_costmap_3d/costmap_publisher.hpp"
#include <octomap_msgs/conversions.h>

namespace nav2_drone_costmap_3d
{

CostmapPublisher::CostmapPublisher(const rclcpp::NodeOptions & options)
: Node("costmap_publisher", options),
  lookahead_dist_(2.0),
  drone_radius_(0.5),
  safety_radius_(0.3),
  octomap_received_(false),
  transform_tolerance_(0.1)
{
  // Initialize TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Declare and retrieve parameters
  declare_parameter("lookahead_dist", lookahead_dist_);
  declare_parameter("drone_radius", drone_radius_);
  declare_parameter("safety_radius", safety_radius_);
  get_parameter("lookahead_dist", lookahead_dist_);
  get_parameter("drone_radius", drone_radius_);
  get_parameter("safety_radius", safety_radius_);

  // Subscriptions
  octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
    "octomap", rclcpp::QoS(10).best_effort(),
    std::bind(&CostmapPublisher::handle_octomap, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::QoS(10),
    std::bind(&CostmapPublisher::handle_odometry, this, std::placeholders::_1));

  // Publisher
  costmap_pub_ = create_publisher<nav2_drone_msgs::msg::CostmapMetaData>(
    "costmap_meta_data", rclcpp::QoS(10));

  // Timer for periodic publishing
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CostmapPublisher::handle_timer, this));
}

void CostmapPublisher::handle_octomap(
  const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);

  std::unique_ptr<octomap::AbstractOcTree> tree_base(
    octomap_msgs::msgToMap(*msg));
  octree_.reset(dynamic_cast<octomap::OcTree *>(tree_base.release()));

  if (octree_) {
    octomap_received_ = true;
    last_octomap_update_ = this->now();
  }
}

void CostmapPublisher::handle_odometry(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_velocity_ = msg->twist.twist;
}

void CostmapPublisher::handle_timer()
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!octomap_received_) {
    RCLCPP_WARN(get_logger(), "Waiting for octomap...");
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  if (!get_drone_pose(pose)) {
    RCLCPP_WARN(get_logger(), "Failed to get drone pose");
    return;
  }

  costmap_.updateCostmap(pose);

  auto metadata_msg = costmap_.getMetadata();
  metadata_msg.header.stamp = now();
  costmap_pub_->publish(metadata_msg);
}

bool CostmapPublisher::get_drone_pose(
  geometry_msgs::msg::PoseStamped & pose) const
{
  const std::string target = map_frame_.empty() ? "map" : map_frame_;
  const std::string source = drone_frame_.empty() ? "base_link" : drone_frame_;
  try {
    auto transform = tf_buffer_->lookupTransform(
      target, source, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "TF error: %s", ex.what());
    return false;
  }
}

}  // namespace nav2_drone_costmap_3d
