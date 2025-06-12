#include "nav2_drone_mpc_controller/histogram.hpp"

namespace nav2_drone_mpc_controller {

Histogram::Histogram(std::size_t resolution)
: resolution_(resolution),
  z_dim_(360u / resolution_),
  e_dim_(180u / resolution_),
  weight_(e_dim_, z_dim_),
  bins_(e_dim_ * z_dim_, 0)
{
  set_zero();
}

const std::vector<std::size_t> & Histogram::getBins() const
{
  return bins_;
}

void Histogram::upsample()   { /* … */ }
void Histogram::downsample() { /* … */ }

void Histogram::set_zero()
{
  weight_.setZero();
  std::fill(bins_.begin(), bins_.end(), 0);
}

bool Histogram::is_empty() const
{
  return weight_.sum() == 0.0f;
}

void Histogram::go_binary(float low, float high)
{
  // Convert weight_ to binary bins_ (0 or 1) using thresholds
  for (std::size_t e = 0; e < e_dim_; ++e) {
    for (std::size_t z = 0; z < z_dim_; ++z) {
      float w = weight_(e, z);
      bins_[e * z_dim_ + z] = (w >= low && w <= high) ? 1 : 0;
    }
  }
}

void Histogram::add_weight(std::size_t e, std::size_t z, float w)
{
  if (e < e_dim_ && z < z_dim_) {
    weight_(e, z) += w;
    bins_[e * z_dim_ + z] = static_cast<std::size_t>(weight_(e, z) > 0.0f);
  }
}

bool Histogram::path_available(std::size_t e, std::size_t z) const
{
  return (e < e_dim_ && z < z_dim_) && (bins_[e * z_dim_ + z] > 0);
}

}  // namespace nav2_drone_mpc_controller
