#ifndef NAV2_DRONE_COSTMAP_3D_COSTMAP_3D_HPP_
#define NAV2_DRONE_COSTMAP_3D_COSTMAP_3D_HPP_

#pragma once

#include <float.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "nav2_drone_msgs/msg/costmap_meta_data.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace nav2_drone_costmap_3d {

const int ALPHA_RES = 6;
const int GRID_LENGTH_Z = 360 / ALPHA_RES;
const int GRID_LENGTH_E = 180 / ALPHA_RES;

class Costmap3D {
  int resolution_;
  int z_dim_;
  int e_dim_;
  Eigen::MatrixXf weight_;

  nav2_drone_msgs::msg::CostmapMetaData metadata_;
  std::vector<uint8_t> data_;

  inline void wrap_index(int &x, int &y) const {
    x = x % e_dim_;
    if (x < 0) x += e_dim_;
    y = y % z_dim_;
    if (y < 0) y += z_dim_;
  }

 public:
  Costmap3D(const int res);
  Costmap3D();
  ~Costmap3D() = default;

  inline int z_dim() { return z_dim_; }
  inline int e_dim() { return e_dim_; }

  inline float get_weight(int x, int y) const {
    wrap_index(x, y);
    return weight_(x, y);
  }

  inline bool path_available(int x, int y) const {
    return weight_(x, y) < 0.001;
  }

  inline void set_weight(int x, int y, float value) { weight_(x, y) = value; }
  inline void add_weight(int x, int y, float value) { weight_(x, y) += value; }

  void upsample();
  void downsample();
  void set_zero();
  bool is_empty() const;
  void go_binary(float theta_low, float theta_high);

  bool updateCostmap(const geometry_msgs::msg::PoseStamped & robot_pose);
  std::string getMapFrame() const;
  nav2_drone_msgs::msg::CostmapMetaData getMetadata() const;
  std::vector<uint8_t> getData() const;
  std::vector<float> getDataFloat() const;
};

}  // namespace nav2_drone_costmap_3d
