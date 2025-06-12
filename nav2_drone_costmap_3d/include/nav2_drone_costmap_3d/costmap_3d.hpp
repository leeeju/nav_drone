#ifndef NAV2_DRONE_COSTMAP_3D_COSTMAP_3D_HPP_
#define NAV2_DRONE_COSTMAP_3D_COSTMAP_3D_HPP_

#include <string>
#include <memory>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "nav2_drone_costmap_3d/costmap_3d.hpp"

namespace nav2_drone_costmap_3d
{

class LayeredCostmap3D
{
public:
  LayeredCostmap3D();
  void updateFromOctomap(const octomap_msgs::msg::Octomap & octo_msg);
  void addPlugin(const std::shared_ptr<rclcpp::Node> & node);
  // ... other plugin management methods ...

private:
  std::mutex mutex_;
  // octomap storage, costmap layers, etc.
};

class CostmapPublisher : public rclcpp::Node
{
public:
  explicit CostmapPublisher(const rclcpp::NodeOptions & options);
  ~CostmapPublisher() override = default;

private:
  // Subscribers & timers
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string octomap_topic_;
  std::string odom_topic_;
  double publish_rate_;

  // State
  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_pose_;
  bool have_odom_{false};

  // Costmap
  std::shared_ptr<LayeredCostmap3D> layered_costmap_;

  // Callbacks
  void handleOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onTimer();
};

}  // namespace nav2_drone_costmap_3d

#endif  // NAV2_DRONE_COSTMAP_3D_COSTMAP_3D_HPP_
