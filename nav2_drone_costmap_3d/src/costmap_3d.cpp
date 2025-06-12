#include "nav2_drone_costmap_3d/costmap_3d.hpp"

using namespace nav2_drone_costmap_3d;

LayeredCostmap3D::LayeredCostmap3D()
{
  // Constructor: initialize octomap and layer structures
}

void LayeredCostmap3D::updateFromOctomap(const octomap_msgs::msg::Octomap & octo_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Convert and integrate octo_msg into internal layers
}

void LayeredCostmap3D::addPlugin(const std::shared_ptr<rclcpp::Node> & node)
{
  // Optionally register this costmap with a broader framework
}

CostmapPublisher::CostmapPublisher(const rclcpp::NodeOptions & options)
: Node("costmap_publisher_3d", options)
{
  // Declare and get parameters
  this->declare_parameter("octomap_topic", std::string("/octomap_binary"));
  this->declare_parameter("odom_topic", std::string("/odom"));
  this->declare_parameter("publish_rate", 1.0);

  this->get_parameter("octomap_topic", octomap_topic_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("publish_rate", publish_rate_);

  // Initialize layered costmap
  layered_costmap_ = std::make_shared<LayeredCostmap3D>();

  // Subscribers
  octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, 10,
    std::bind(&CostmapPublisher::handleOctomap, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10,
    std::bind(&CostmapPublisher::handleOdometry, this, std::placeholders::_1));

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_),
    std::bind(&CostmapPublisher::onTimer, this));
}

void CostmapPublisher::handleOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  layered_costmap_->updateFromOctomap(*msg);
}

void CostmapPublisher::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Update last known pose
  last_pose_.header = msg->header;
  last_pose_.pose = msg->pose.pose;
  have_odom_ = true;
}

void CostmapPublisher::onTimer()
{
  if (!have_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry received yet");
    return;
  }

  // Publish or process the costmap based on layered_costmap_ and last_pose_
  RCLCPP_INFO(this->get_logger(), "Publishing 3D costmap at pose (%.2f, %.2f, %.2f)",
    last_pose_.pose.position.x,
    last_pose_.pose.position.y,
    last_pose_.pose.position.z);
}
