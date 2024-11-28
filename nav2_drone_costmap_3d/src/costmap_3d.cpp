#include "costmap_3d.hpp"

namespace nav_drone_costmap_3d {

Costmap3D::Costmap3D(float resolution, float default_cost)
    : resolution_(resolution), z_dim_(GRID_LENGTH_Z), e_dim_(GRID_LENGTH_E) {
  costmap_ = Eigen::MatrixXf::Constant(e_dim_, z_dim_, default_cost);
}

void Costmap3D::wrapIndex(int &elevation_idx, int &azimuth_idx) const {
  elevation_idx = elevation_idx % e_dim_;
  if (elevation_idx < 0) elevation_idx += e_dim_;
  azimuth_idx = azimuth_idx % z_dim_;
  if (azimuth_idx < 0) azimuth_idx += z_dim_;
}

float Costmap3D::getCost(int elevation_idx, int azimuth_idx) const {
  wrapIndex(elevation_idx, azimuth_idx);
  return costmap_(elevation_idx, azimuth_idx);
}

void Costmap3D::setCost(int elevation_idx, int azimuth_idx, float cost) {
  wrapIndex(elevation_idx, azimuth_idx);
  costmap_(elevation_idx, azimuth_idx) = cost;
}

void Costmap3D::addCost(int elevation_idx, int azimuth_idx, float value) {
  wrapIndex(elevation_idx, azimuth_idx);
  costmap_(elevation_idx, azimuth_idx) += value;
}

bool Costmap3D::isPathAvailable(int elevation_idx, int azimuth_idx) const {
  return getCost(elevation_idx, azimuth_idx) < 0.001f;
}

void Costmap3D::reset(float default_cost) {
  costmap_.setConstant(default_cost);
}

void Costmap3D::upsample() {
  Eigen::MatrixXf upsampled = Eigen::MatrixXf::Zero(e_dim_ * 2, z_dim_ * 2);
  for (int e = 0; e < e_dim_; ++e) {
    for (int z = 0; z < z_dim_; ++z) {
      float cost = costmap_(e, z);
      upsampled(2 * e, 2 * z) = cost;
      upsampled(2 * e + 1, 2 * z) = cost;
      upsampled(2 * e, 2 * z + 1) = cost;
      upsampled(2 * e + 1, 2 * z + 1) = cost;
    }
  }
  costmap_ = upsampled;
  e_dim_ *= 2;
  z_dim_ *= 2;
}

void Costmap3D::downsample() {
  Eigen::MatrixXf downsampled = Eigen::MatrixXf::Zero(e_dim_ / 2, z_dim_ / 2);
  for (int e = 0; e < e_dim_; e += 2) {
    for (int z = 0; z < z_dim_; z += 2) {
      downsampled(e / 2, z / 2) = costmap_(e, z);
    }
  }
  costmap_ = downsampled;
  e_dim_ /= 2;
  z_dim_ /= 2;
}

void Costmap3D::binarize(float threshold) {
  for (int e = 0; e < e_dim_; ++e) {
    for (int z = 0; z < z_dim_; ++z) {
      costmap_(e, z) = (costmap_(e, z) > threshold) ? 1.0f : 0.0f;
    }
  }
}

bool Costmap3D::isEmpty() const {
  return costmap_.isZero(0.001f);
}

}  // namespace nav_drone_costmap_3d
