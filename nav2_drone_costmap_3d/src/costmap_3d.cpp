#include "nav2_drone_costmap_3d/costmap_3d.hpp"
#include <rclcpp/rclcpp.hpp>

namespace nav2_drone_costmap_3d {

Costmap3D::Costmap3D()
: metadata_(),
  data_()
{
  // 기본 생성자: metadata_와 data_를 기본값으로 초기화
}

bool Costmap3D::updateCostmap(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // 로봇의 현재 포즈를 메타데이터 헤더에 반영
  metadata_.header = robot_pose.header;

  // TODO: octo_tree_ 또는 외부 데이터로부터 costmap data_ 갱신 로직 구현
  // 예: data_.assign(...) 또는 외부 API 호출

  return true;
}

std::string Costmap3D::getMapFrame() const
{
  // 메타데이터 헤더의 frame_id 반환
  return metadata_.header.frame_id;
}

nav2_drone_msgs::msg::CostmapMetaData Costmap3D::getMetadata() const
{
  // 현재 메타데이터 반환
  return metadata_;
}

std::vector<uint8_t> Costmap3D::getData() const
{
  // 원시 바이트 데이터 반환
  return data_;
}

std::vector<float> Costmap3D::getDataFloat() const
{
  // 바이트 데이터를 float 벡터로 변환하여 반환
  std::vector<float> out;
  out.reserve(data_.size());
  for (uint8_t v : data_) {
    out.push_back(static_cast<float>(v));
  }
  return out;
}

}  // namespace nav2_drone_costmap_3d
