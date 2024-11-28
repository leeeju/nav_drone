#ifndef NAV2_DRONE_COSTMAP_3D__COSTMAP_3D_HPP_
#define NAV2_DRONE_COSTMAP_3D__COSTMAP_3D_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_drone_msgs/msg/costmap_meta_data.hpp"

namespace nav2_drone_costmap_3d {

class Costmap3D
{
public:
  Costmap3D();

  bool updateCostmap(const geometry_msgs::msg::PoseStamped &robot_pose);
  std::string getMapFrame() const;
  nav2_drone_msgs::msg::CostmapMetaData getMetadata() const;
  std::vector<uint8_t> getData() const;

private:
  std::string map_frame_;
  nav2_drone_msgs::msg::CostmapMetaData metadata_;
  std::vector<uint8_t> data_;
  double depth_;
  double size_z_;
};

}  // namespace nav2_drone_costmap_3d

#endif  // NAV2_DRONE_COSTMAP_3D__COSTMAP_3D_HPP_
