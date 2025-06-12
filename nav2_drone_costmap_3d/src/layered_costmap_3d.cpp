#include "nav2_drone_costmap_3d/layered_costmap_3d.hpp"

namespace nav2_drone_costmap_3d
{

LayeredCostmap3D::LayeredCostmap3D() = default;

LayeredCostmap3D::~LayeredCostmap3D() = default;

void LayeredCostmap3D::addLayer(std::shared_ptr<Layer> layer)
{
  layers_.push_back(std::move(layer));
}

void LayeredCostmap3D::update()
{
  for (auto & layer : layers_) {
    layer->update(master_costmap_);
  }
}

Costmap3D & LayeredCostmap3D::getMasterCostmap()
{
  return master_costmap_;
}

}  // namespace nav2_drone_costmap_3d
