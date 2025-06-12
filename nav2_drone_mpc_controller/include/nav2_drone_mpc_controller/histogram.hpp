#ifndef NAV2_DRONE_MPC_CONTROLLER__HISTOGRAM_HPP_
#define NAV2_DRONE_MPC_CONTROLLER__HISTOGRAM_HPP_

#include <vector>
#include <cstddef>
#include <Eigen/Dense>

namespace nav2_drone_mpc_controller {

class Histogram {
public:
  explicit Histogram(std::size_t resolution);

  const std::vector<std::size_t> & getBins() const;
  void upsample();
  void downsample();
  void set_zero();
  bool is_empty() const;
  void go_binary(float low, float high);

  // Helpers
  std::size_t z_dim() const { return z_dim_; }
  std::size_t e_dim() const { return e_dim_; }
  void add_weight(std::size_t e, std::size_t z, float w);
  bool path_available(std::size_t e, std::size_t z) const;

private:
  std::size_t resolution_;
  std::size_t z_dim_;
  std::size_t e_dim_;
  Eigen::MatrixXf weight_;
  std::vector<std::size_t> bins_;
};

}  // namespace nav2_drone_mpc_controller

#endif  // NAV2_DRONE_MPC_CONTROLLER__HISTOGRAM_HPP_
