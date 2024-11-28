#ifndef NAV2_DRONE_COSTMAP_3D_LAYERED_COSTMAP_3D_HPP_
#define NAV2_DRONE_COSTMAP_3D_LAYERED_COSTMAP_3D_HPP_

#include <vector>
#include <memory>
#include "nav2_drone_costmap_3d/costmap_3d.hpp"

// Forward declaration of Costmap3D
class Costmap3D;

namespace nav2_drone_costmap_3d
{

class Layer
{
public:
  virtual ~Layer() = default;

  // Update the layer with new data
  virtual void update(Costmap3D &master_costmap) = 0;
};

class LayeredCostmap3D
{
public:
  LayeredCostmap3D();
  ~LayeredCostmap3D();

  // Add a new layer to the costmap
  void addLayer(std::shared_ptr<Layer> layer);

  // Update all layers and combine into the master costmap
  void update();

  // Get the master costmap
  Costmap3D &getMasterCostmap();

private:
  Costmap3D master_costmap_;                  // Master 3D costmap
  std::vector<std::shared_ptr<Layer>> layers_; // List of layers
};

}  // namespace nav2_drone_costmap_3d

#endif  // NAV2_DRONE_COSTMAP_3D_LAYERED_COSTMAP_3D_HPP_
