#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <geometry_msgs/msg/point.hpp>

namespace nav2_drone_costmap_3d {

// Valid resolutions for ALPHA_RES must satisfy: 180 % (2 * ALPHA_RES) = 0
// Examples of valid values: 1, 3, 5, 6, 10, 15, 18, 30, 45, 60
const int ALPHA_RES = 6;  // Angular resolution in degrees
const int GRID_LENGTH_Z = 360 / ALPHA_RES;  // Azimuth range (360 degrees)
const int GRID_LENGTH_E = 180 / ALPHA_RES;  // Elevation range (180 degrees)

// Costmap3D class: Optimized for managing a 3D spherical costmap
class Costmap3D {
public:
  // Constructor: Initialize costmap with resolution and default cost
  explicit Costmap3D(float resolution, float default_cost = 0.0f);

  // Destructor
  ~Costmap3D() = default;

  // Accessor for azimuth (Z-axis) dimension
  inline int z_dim() const { return z_dim_; }

  // Accessor for elevation (E-axis) dimension
  inline int e_dim() const { return e_dim_; }

  // Get the cost at a specific elevation and azimuth index
  float getCost(int elevation_idx, int azimuth_idx) const;

  // Set the cost at a specific elevation and azimuth index
  void setCost(int elevation_idx, int azimuth_idx, float cost);

  // Add a weight (cost increment) to a specific cell
  void addCost(int elevation_idx, int azimuth_idx, float value);

  // Check if a path is available at a specific cell
  inline bool isPathAvailable(int elevation_idx, int azimuth_idx) const;

  // Reset all costs in the costmap to the default value
  void reset(float default_cost = 0.0f);

  // Upsample the costmap: Increase resolution
  void upsample();

  // Downsample the costmap: Reduce resolution
  void downsample();

  // Translate all costs into binary values (0 or 1)
  void binarize(float threshold);

  // Check if the costmap is empty (no significant costs)
  bool isEmpty() const;

private:
  float resolution_;               // Resolution of the costmap (meters per cell)
  int z_dim_;                      // Number of azimuth cells
  int e_dim_;                      // Number of elevation cells
  Eigen::MatrixXf costmap_;        // Eigen matrix to store cost values

  // Helper: Wrap elevation and azimuth indices into valid ranges
  inline void wrapIndex(int &elevation_idx, int &azimuth_idx) const;
};

}  // namespace nav2_drone_costmap_3d
