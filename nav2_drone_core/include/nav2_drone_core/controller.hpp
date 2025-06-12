#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_drone_costmap_3d/costmap_server.hpp"
#include "nav2_drone_costmap_3d/layered_costmap_3d.hpp"

namespace nav2_drone_core
{

/**
 * @class Controller
 * @brief Base interface for implementing navigation controllers
 */
class Controller
{
public:
  using Ptr = std::shared_ptr<Controller>;

  /**
   * @brief Configure the controller with necessary parameters and dependencies
   * @param parent Shared pointer to the parent node
   * @param name Name of the controller
   * @param costmap Shared pointer to the 3D costmap publisher
   */
  virtual void configure(
    const rclcpp::Node::SharedPtr parent,
    const std::string & name,
    const std::shared_ptr<nav2_drone_costmap_3d::CostmapPublisher> & costmap) = 0;

  /**
   * @brief Set the global path for the controller to follow
   * @param path The navigation path
   */
  virtual void setPath(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief Update the costmap data to reflect the latest map changes
   * @param costmap Shared pointer to the updated costmap publisher
   */
  virtual void updateMap(const std::shared_ptr<nav2_drone_costmap_3d::CostmapPublisher> & costmap) = 0;

  /**
   * @brief Compute velocity commands for navigation based on the current pose and speed
   * @param pose The current pose of the drone
   * @param speed The current velocity of the drone
   * @return A stamped velocity command
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed) = 0;

  /**
   * @brief Virtual destructor
   */
  virtual ~Controller() = default;

protected:
  /**
   * @brief Protected constructor to prevent direct instantiation
   */
  Controller() = default;
};

}  // namespace nav2_drone_core
