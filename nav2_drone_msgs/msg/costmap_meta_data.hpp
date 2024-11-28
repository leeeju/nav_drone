#ifndef NAV2_DRONE_MSGS__MSG__COSTMAP_META_DATA_HPP_
#define NAV2_DRONE_MSGS__MSG__COSTMAP_META_DATA_HPP_

#include <string>
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace nav2_drone_msgs
{
namespace msg
{

/**
 * @class CostmapMetaData
 * @brief This class represents metadata for a costmap.
 */
class CostmapMetaData
{
public:
  // Constructor
  CostmapMetaData()
  : resolution(0.0),
    width(0.0),
    height(0.0),
    depth(0.0),
    size_z(0.0) {}

  // Destructor
  virtual ~CostmapMetaData() = default;

  // Header for timestamp and frame information
  std_msgs::msg::Header header;

  // Resolution of the costmap in meters/cell
  double resolution;

  // Width, height, and depth of the costmap
  double width;
  double height;
  double depth;
  double size_z;

  // Origin of the costmap in the global frame
  geometry_msgs::msg::Pose origin;  // Pose with position and orientation
};

}  // namespace msg
}  // namespace nav2_drone_msgs

#endif  // NAV2_DRONE_MSGS__MSG__COSTMAP_META_DATA_HPP_
