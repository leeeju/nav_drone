#include "nav2_drone_costmap_3d/octomap_wrapper.hpp"
#include <algorithm>

namespace nav2_drone_costmap_3d
{

void OctoMapWrapper::updateFromOctomap(
  const octomap_msgs::msg::Octomap & msg)
{
  data_.clear();
  data_.reserve(msg.data.size());
  for (const auto & v : msg.data) {
    data_.push_back(static_cast<uint8_t>(v));
  }
  // OctoMap 트리 변환
  std::unique_ptr<octomap::AbstractOcTree> tree_base(
    octomap_msgs::msgToMap(msg));
  tree_ = std::dynamic_pointer_cast<octomap::OcTree>(
    std::shared_ptr<octomap::AbstractOcTree>(std::move(tree_base)));
}

std::vector<float> OctoMapWrapper::getDataFloat() const
{
  std::vector<float> out;
  out.reserve(data_.size());
  std::transform(data_.begin(), data_.end(), std::back_inserter(out),
    [](uint8_t v) { return static_cast<float>(v); });
  return out;
}

std::shared_ptr<const octomap::OcTree> OctoMapWrapper::getOctree() const
{
  return tree_;
}

}  // namespace nav2_drone_costmap_3d
