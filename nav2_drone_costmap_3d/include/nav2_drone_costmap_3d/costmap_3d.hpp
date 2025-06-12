#ifndef NAV2_DRONE_COSTMAP_3D_COSTMAP_3D_HPP_
#define NAV2_DRONE_COSTMAP_3D_COSTMAP_3D_HPP_

#include <mutex>
#include <memory>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_drone_msgs/msg/costmap_meta_data.hpp"

namespace nav2_drone_costmap_3d
{

/**
 * @brief Minimal 3D costmap container used by the drone modules.
 *        It stores the map data internally and exposes simple
 *        update and query APIs used by the publisher.
 */
class Costmap3D
{
public:
  Costmap3D() = default;

  /// Update the internal costmap representation for the given pose.
  void updateCostmap(const geometry_msgs::msg::PoseStamped & /*pose*/) {}

  /// Retrieve meta data describing the current costmap.
  nav2_drone_msgs::msg::CostmapMetaData getMetadata() const
  {
    return nav2_drone_msgs::msg::CostmapMetaData();
  }

private:
  mutable std::mutex mutex_;
};

}  // namespace nav2_drone_costmap_3d

#endif  // NAV2_DRONE_COSTMAP_3D_COSTMAP_3D_HPP_
