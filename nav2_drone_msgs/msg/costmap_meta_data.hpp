#ifndef NAV2_DRONE_MSGS__MSG__COSTMAP_META_DATA_HPP_
#define NAV2_DRONE_MSGS__MSG__COSTMAP_META_DATA_HPP_

#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace nav2_drone_msgs
{
namespace msg
{

struct CostmapMetaData
{
  using SharedPtr = std::shared_ptr<CostmapMetaData>;
  using ConstSharedPtr = std::shared_ptr<const CostmapMetaData>;

  std_msgs::msg::Header header;
  double resolution = 0.0;
  double width = 0.0;
  double height = 0.0;
  double size_z = 0.0;
  double depth = 0.0;
  geometry_msgs::msg::Pose origin;
};

}  // namespace msg
}  // namespace nav2_drone_msgs

#endif  // NAV2_DRONE_MSGS__MSG__COSTMAP_META_DATA_HPP_
