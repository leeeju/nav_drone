#include "nav2_drone_costmap_3d/costmap_publisher.hpp"

namespace nav2_drone_costmap_3d {

CostmapPublisher::CostmapPublisher(const std::string &name)
: Node(name), current_(false), octomap_(nullptr), last_octomap_update_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
  costmap_()
{
  init();
}

void CostmapPublisher::init()
{
  // TODO: Initialize publishers, subscribers, and parameters
}

}  // namespace nav2_drone_costmap_3d
