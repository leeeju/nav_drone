#ifndef NAV2_DRONE_COSTMAP_3D__COSTMAP_PUBLISHER_HPP_
#define NAV2_DRONE_COSTMAP_3D__COSTMAP_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_drone_costmap_3d/costmap_3d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "octomap_msgs/msg/octomap.hpp"

namespace nav2_drone_costmap_3d {

class CostmapPublisher : public rclcpp::Node
{
public:
  explicit CostmapPublisher(const std::string &name);

private:
  void init();

  // Member variables
  bool current_;
  std::shared_ptr<octomap::OcTree> octomap_;
  rclcpp::Time last_octomap_update_;

  Costmap3D costmap_;
};

}  // namespace nav2_drone_costmap_3d

#endif  // NAV2_DRONE_COSTMAP_3D__COSTMAP_PUBLISHER_HPP_
