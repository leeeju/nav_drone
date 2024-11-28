#include "nav2_drone_costmap_3d/costmap_3d.hpp"

namespace nav2_drone_costmap_3d {

Costmap3D::Costmap3D()
: map_frame_("map"), metadata_(), data_(), depth_(0.0), size_z_(0.0)
{
  metadata_.resolution = 0.05;
  metadata_.width = 0.0;
  metadata_.height = 0.0;
  metadata_.depth = 0.0;
  metadata_.size_z = 0.0;

  metadata_.origin.position.x = 0.0;
  metadata_.origin.position.y = 0.0;
  metadata_.origin.position.z = 0.0;
  metadata_.origin.orientation.w = 1.0;
  metadata_.origin.orientation.x = 0.0;
  metadata_.origin.orientation.y = 0.0;
  metadata_.origin.orientation.z = 0.0;

  data_.clear();
}

bool Costmap3D::updateCostmap(const geometry_msgs::msg::PoseStamped &robot_pose) {
  RCLCPP_INFO(rclcpp::get_logger("Costmap3D"), "Updating costmap for robot pose at [%f, %f, %f]",
              robot_pose.pose.position.x,
              robot_pose.pose.position.y,
              robot_pose.pose.position.z);

  return true;
}

std::string Costmap3D::getMapFrame() const {
  return map_frame_;
}

nav2_drone_msgs::msg::CostmapMetaData Costmap3D::getMetadata() const {
  auto updated_metadata = metadata_;
  updated_metadata.depth = depth_;
  updated_metadata.size_z = size_z_;
  return updated_metadata;
}

std::vector<uint8_t> Costmap3D::getData() const {
  return data_;
}

}  // namespace nav2_drone_costmap_3d
