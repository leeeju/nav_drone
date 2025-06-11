#pragma once

#include <vector>
#include <cstdint>
#include <memory>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include "octomap_msgs/msg/octomap.hpp"

namespace nav2_drone_costmap_3d {

class OctoMapWrapper
{
public:
  void updateFromOctomap(
    const octomap_msgs::msg::Octomap & msg);
  std::vector<float> getDataFloat() const;
  std::shared_ptr<const octomap::OcTree> getOctree() const;

private:
  std::vector<uint8_t> data_;
  std::shared_ptr<octomap::OcTree> tree_;
};

}  // namespace nav2_drone_costmap_3d
