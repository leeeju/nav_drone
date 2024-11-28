#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>  // std::mutex
#include <thread> // std::thread for asynchronous operations

#include "nav2_drone_costmap_3d/costmap_publisher.hpp"
#include "nav2_drone_costmap_3d/costmap_server.hpp"
#include "nav2_drone_costmap_3d/costmap_3d.hpp"
#include "nav2_drone_msgs/msg/costmap.hpp"
// #include "nav2_drone_costmap_3d/cost_values.hpp"

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

using namespace std::chrono_literals;

namespace nav2_drone_costmap_3d {

class CostmapServer : public rclcpp::Node {
public:
  /**
   * @brief Constructor for CostmapServer
   * @param node_name The name of the costmap server node
   */
  explicit CostmapServer(const std::string &node_name = "costmap_server");

  /**
   * @brief Initialize the server's parameters and subscriptions
   */
  void initialize();

  /**
   * @brief Updates the costmap based on the current robot pose and surroundings
   */
  void updateCostmap();

  /**
   * @brief Retrieves the robot's pose in the global frame
   * @param[out] global_pose The robot's global pose
   * @return true if the pose was successfully retrieved
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped &global_pose);

  /**
   * @brief Publish the updated costmap at a regular interval
   */
  void publishCostmap();

  /**
   * @brief Returns the cost at a given pose
   * @param[in] pose The queried pose
   * @return The cost at the specified pose
   */
  unsigned char getCostAtPose(const geometry_msgs::msg::PoseStamped &pose);

  /**
   * @brief Reset the costmap to its default state
   */
  void resetCostmap();

protected:
  // Parameters
  std::string map_frame_;          // Global frame ID
  std::string robot_base_frame_;   // Robot's base frame ID
  double transform_tolerance_;     // TF2 transform tolerance
  double robot_radius_;            // Radius of the robot
  double safety_radius_;           // Safety buffer radius
  double lookahead_distance_;      // Lookahead distance for path planning
  bool use_velocity_scaled_lookahead_; // Use velocity-scaled lookahead distance

  // State variables
  bool is_current_;                // Indicates if the costmap is up-to-date

  // Mutex for thread-safe costmap updates
  std::mutex costmap_mutex_;

  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr update_timer_;

private:
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Costmap and Octomap
  std::shared_ptr<Costmap3D> costmap_;
  std::shared_ptr<octomap::OcTree> octomap_;

  // Subscriptions
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  // Publishers
  rclcpp::Publisher<nav2_drone_msgs::msg::Costmap>::SharedPtr costmap_publisher_;

  // Helper functions
  void mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  std::pair<int, int> getGridPosition(const octomap::point3d &point);
  double computeLookaheadDistance(const geometry_msgs::msg::Twist &velocity);
};

}  // namespace nav2_drone_costmap_3d
