#ifndef NAV2_DRONE_COSTMAP_3D__COSTMAP_SERVER_HPP_
#define NAV2_DRONE_COSTMAP_3D__COSTMAP_SERVER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace nav2_drone_costmap_3d
{

/**
 * @class CostmapPublisher
 * @brief 3D OctoMap-based costmap layer for drone navigation
 */
class CostmapPublisher : public nav2_costmap_2d::CostmapLayer,
                        public std::enable_shared_from_this<CostmapPublisher>
{
public:
  CostmapPublisher();
  ~CostmapPublisher() override;

  /**
   * @brief Lifecycle initialization callback
   */
  void onInitialize() override;

  /**
   * @brief Reset the layer state
   */
  void reset() override;

  /**
   * @brief Update the bounding box for costmap updates
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double* min_x, double* min_y,
    double* max_x, double* max_y) override;

  /**
   * @brief Update the cost values within the bounding box
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j,
    int max_i, int max_j) override;

private:
  // Parameter for odometry topic
  std::string odom_topic_;

  // Subscription to odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Mutex for shared data
  std::mutex mutex_;

  // Last received odometry message
  nav_msgs::msg::Odometry::SharedPtr last_odom_;

  /**
   * @brief Odometry callback
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace nav2_drone_costmap_3d

#endif  // NAV2_DRONE_COSTMAP_3D__COSTMAP_SERVER_HPP_
