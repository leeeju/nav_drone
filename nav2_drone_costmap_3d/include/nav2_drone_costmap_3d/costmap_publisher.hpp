#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_drone_msgs/msg/costmap_meta_data.hpp"
#include "nav2_drone_costmap_3d/costmap_3d.hpp"
#include <octomap/OcTree.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace nav2_drone_costmap_3d
{

class CostmapPublisher : public rclcpp::Node
{
public:
  explicit CostmapPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  bool isCurrent() const {return octomap_received_;}
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose) const {return get_drone_pose(pose);} 
  double getTransformTollerance() const {return transform_tolerance_;}
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() {return tf_buffer_;}
  std::string getGlobalFrameID() const {return map_frame_;}

private:
  // Callback for incoming OCTOMAP messages
  void handle_octomap(const octomap_msgs::msg::Octomap::SharedPtr msg);
  // Callback for incoming odometry messages
  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  // Timer-based publish
  void handle_timer();
  // Helper to lookup drone pose via TF
  bool get_drone_pose(geometry_msgs::msg::PoseStamped & pose) const;

  // Parameters
  double lookahead_dist_;
  double drone_radius_;
  double safety_radius_;

  // ROS interfaces
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav2_drone_msgs::msg::CostmapMetaData>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Internal state
  Costmap3D costmap_;
  std::mutex costmap_mutex_;
  geometry_msgs::msg::Twist last_velocity_;
  std::unique_ptr<octomap::OcTree> octree_;
  rclcpp::Time last_octomap_update_;
  bool octomap_received_;

  // TF components
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_;
  std::string drone_frame_;
  double transform_tolerance_;
};

}  // namespace nav2_drone_costmap_3d
