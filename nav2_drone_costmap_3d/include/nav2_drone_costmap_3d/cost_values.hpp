#ifndef NAV2_DRONE_COSTMAP_3D__COST_VALUES_HPP_
#define NAV2_DRONE_COSTMAP_3D__COST_VALUES_HPP_

/**
 * @file cost_values.hpp
 * @brief Defines common cost values used in the costmap
 */

namespace nav2_drone_costmap_3d
{
// Represents unknown or uninitialized space in the costmap
inline constexpr unsigned char NO_INFORMATION = 255;

// Represents a lethal obstacle in the costmap
inline constexpr unsigned char LETHAL_OBSTACLE = 254;

// Represents an inscribed inflated obstacle (near-lethal zone)
inline constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;

// Represents the maximum value for a non-obstacle space
inline constexpr unsigned char MAX_NON_OBSTACLE = 252;

// Represents free or open space in the costmap
inline constexpr unsigned char FREE_SPACE = 0;

}  // namespace nav2_drone_costmap_3d

#endif  // NAV2_DRONE_COSTMAP_3D__COST_VALUES_HPP_
