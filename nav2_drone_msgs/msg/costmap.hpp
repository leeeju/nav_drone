#ifndef NAV2_DRONE_MSGS__MSG__COSTMAP_HPP_
#define NAV2_DRONE_MSGS__MSG__COSTMAP_HPP_

#include <vector>
#include "std_msgs/msg/header.hpp"
#include "nav2_drone_msgs/msg/costmap_meta_data.hpp"

namespace nav2_drone_msgs
{
namespace msg
{

/**
 * @class Costmap
 * @brief Represents a costmap with metadata and cost values.
 */
class Costmap
{
public:
  // Constructor
  Costmap() = default;

  // Destructor
  virtual ~Costmap() = default;

  // Header for timestamp and frame information
  std_msgs::msg::Header header;

  // Metadata for the costmap
  nav2_drone_msgs::msg::CostmapMetaData metadata;

  // The cost data, in row-major order, starting with (0,0)
  std::vector<uint8_t> data;
};

}  // namespace msg
}  // namespace nav2_drone_msgs

#endif  // NAV2_DRONE_MSGS__MSG__COSTMAP_HPP_
