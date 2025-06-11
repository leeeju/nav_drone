#include "nav2_drone_costmap_3d/costmap_server.hpp"

namespace nav2_drone_costmap_3d
{

CostmapPublisher::CostmapPublisher()
: nav2_costmap_2d::CostmapLayer()
{
  // Enable shared_from_this
}

CostmapPublisher::~CostmapPublisher() = default;

void CostmapPublisher::onInitialize()
{
  // Lock the lifecycle node
  auto shared_node = node_.lock();
  // Declare and get parameters
  shared_node->declare_parameter("odom_topic", std::string("/odom"));
  shared_node->get_parameter("odom_topic", odom_topic_);

  // Subscribe to odometry
  odom_sub_ = shared_node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&CostmapPublisher::odomCallback, this, std::placeholders::_1)
  );

  // Add this layer to the layered costmap
  layered_costmap_->addPlugin(shared_from_this());
}

void CostmapPublisher::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_odom_.reset();
}

void CostmapPublisher::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double* min_x, double* min_y, double* max_x, double* max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_odom_) {
    return;
  }
  // Expand bounds around last known position
  double x = last_odom_->pose.pose.position.x;
  double y = last_odom_->pose.pose.position.y;

  *min_x = std::min(*min_x, x - 1.0);
  *min_y = std::min(*min_y, y - 1.0);
  *max_x = std::max(*max_x, x + 1.0);
  *max_y = std::max(*max_y, y + 1.0);
}

void CostmapPublisher::updateCosts(
  nav2_costmap_2d::Costmap2D& master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_odom_) {
    return;
  }

  unsigned int mx, my;
  double x = last_odom_->pose.pose.position.x;
  double y = last_odom_->pose.pose.position.y;

  // Convert world to map coordinates
  if (master_grid.worldToMap(x, y, mx, my)) {
    // Mark obstacle at drone position
    master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}

void CostmapPublisher::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_odom_ = msg;
}

}  // namespace nav2_drone_costmap_3d
