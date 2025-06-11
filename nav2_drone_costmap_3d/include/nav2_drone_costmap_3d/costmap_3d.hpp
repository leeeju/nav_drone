#pragma once

#include <vector>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_drone_msgs/msg/costmap_meta_data.hpp>

namespace nav2_drone_costmap_3d {

/**
 * @class Costmap3D
 * @brief 3D costmap representation with metadata and raw data storage.
 */
class Costmap3D
{
public:
  /**
   * @brief Default constructor
   */
  Costmap3D();

  /**
   * @brief Update the costmap based on the robot's pose.
   * @param drone_pose Current robot pose in map frame.
   * @return True if update succeeded, false otherwise.
   */
  bool updateCostmap(const geometry_msgs::msg::PoseStamped & drone_pose);

  /**
   * @brief Get the TF frame of the costmap's map.
   * @return Name of the map frame.
   */
  std::string getMapFrame() const;

  /**
   * @brief Retrieve the costmap metadata.
   * @return A CostmapMetaData message containing resolution, size, origin, etc.
   */
  nav2_drone_msgs::msg::CostmapMetaData getMetadata() const;

  /**
   * @brief Retrieve the raw cost data as bytes (0–255).
   * @return Vector of uint8_t values representing cost.
   */
  std::vector<uint8_t> getData() const;

  /**
   * @brief Retrieve the cost data as floats for publishing.
   * Converts each byte value to float.
   * @return Vector of float values representing cost.
   */
  std::vector<float> getDataFloat() const;

private:
  nav2_drone_msgs::msg::CostmapMetaData metadata_;  ///< Costmap metadata
  std::vector<uint8_t> data_;                     ///< Flattened 3D cost data
};

}  // namespace nav2_drone_costmap_3d
