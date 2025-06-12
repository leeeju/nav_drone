#ifndef NAV2_DRONE_COSTMAP_3D_COSTMAP_PUBLISHER_HPP_
#define NAV2_DRONE_COSTMAP_3D_COSTMAP_PUBLISHER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "octomap/OcTree.h"

#include "nav2_drone_costmap_3d/costmap_3d.hpp"
#include "nav2_drone_msgs/msg/costmap_meta_data.hpp"

namespace nav2_drone_costmap_3d
{

/**
 * @brief Node publishing 3D costmap meta data derived from Octomap input.
 */
class CostmapPublisher : public rclcpp::Node
{
public:
  explicit CostmapPublisher(const rclcpp::NodeOptions & options);

  /// Get the underlying Octomap tree if available
  std::shared_ptr<octomap::OcTree> get_octree() const { return octree_; }

private:
  // Parameters
  double lookahead_dist_;
  double drone_radius_;
  double safety_radius_;
  double transform_tolerance_;
  std::string map_frame_;
  std::string drone_frame_;

  // Subscriptions and publications
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav2_drone_msgs::msg::CostmapMetaData>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // TF2 utilities
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Stored map data
  std::shared_ptr<octomap::OcTree> octree_;
  rclcpp::Time last_octomap_update_;
  geometry_msgs::msg::Twist last_velocity_;
  bool octomap_received_;
  mutable std::mutex costmap_mutex_;

  Costmap3D costmap_;

  // Internal callbacks
  void handle_octomap(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void handle_timer();
  bool get_drone_pose(geometry_msgs::msg::PoseStamped & pose) const;
};

}  // namespace nav2_drone_costmap_3d

#endif  // NAV2_DRONE_COSTMAP_3D_COSTMAP_PUBLISHER_HPP_
