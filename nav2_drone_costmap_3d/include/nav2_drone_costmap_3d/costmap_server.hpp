#ifndef NAV2_DRONE_COSTMAP_3D__COSTMAP_SERVER_HPP_
#define NAV2_DRONE_COSTMAP_3D__COSTMAP_SERVER_HPP_

#include <string>
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "layered_costmap_3d.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_drone_costmap_3d
{

/**
 * @class CostmapServer
 * @brief 3D costmap layer for drones, combining 2D CostmapLayer interface
 *        with 3D layered map representation
 */
class CostmapServer : public nav2_costmap_2d::CostmapLayer,
                      public LayeredCostmap3D
{
public:
  /**
   * @brief Default constructor
   */
  CostmapServer();

  /**
   * @brief Destructor
   */
  virtual ~CostmapServer() = default;

  /**
   * @brief Initialize the costmap layer
   * @param parent pointer to parent ROS node
   */
  void onInitialize() override;

  /**
   * @brief Reset the entire costmap to empty state
   */
  void reset() override;

  /**
   * @brief Update the costmap with the latest sensor data
   */
  void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double & min_x,
    double & min_y,
    double & max_x,
    double & max_y) override;

  /**
   * @brief Populate the master costmap based on this layer's data
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("costmap_3d")};
};

}  // namespace nav2_drone_costmap_3d

#endif  // NAV2_DRONE_COSTMAP_3D__COSTMAP_SERVER_HPP_
