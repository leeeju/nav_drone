#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include "nav2_drone_costmap_3d/layered_costmap_3d.hpp"

namespace nav2_drone_costmap_3d
{

/**
 * @class CostmapPublisher
 * @brief ROS2 node to publish 3D costmap data as Octomap messages
 */
class CostmapPublisher : public rclcpp::Node
{
public:
  /**
   * @brief Constructor, sets up publishers and timer
   * @param options Node options for ROS2 component
   */
  CostmapPublisher(const rclcpp::NodeOptions & options)
  : Node("costmap_publisher", options)
  {
    // Initialize layered costmap
    layered_map_ = std::make_shared<LayeredCostmap3D>(get_parameter("resolution").as_int());

    // Create Octomap publisher
    octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
      "~/octomap", rclcpp::SystemDefaultsQoS());

    // Timer to periodically publish costmap
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CostmapPublisher::publishCallback, this));
  }

private:
  /// Shared pointer to the layered 3D costmap
  std::shared_ptr<LayeredCostmap3D> layered_map_;

  /// ROS publisher for Octomap messages
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;

  /// Timer for periodic publication
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Timer callback to publish the Octomap
   */
  void publishCallback()
  {
    auto msg = octomap_msgs::msg::Octomap();
    // Convert layered_map_ to octomap and fill msg
    // layered_map_->toOctomap(msg);  // Assume LayeredCostmap3D provides this

    msg.header.stamp = now();
    msg.header.frame_id = "map";
    octomap_pub_->publish(msg);
  }
};

}  // namespace nav2_drone_costmap_3d
