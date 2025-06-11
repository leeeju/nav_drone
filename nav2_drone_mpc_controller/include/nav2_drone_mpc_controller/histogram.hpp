#ifndef NAV2_DRONE_MPC_CONTROLLER__HISTOGRAM_HPP_
#define NAV2_DRONE_MPC_CONTROLLER__HISTOGRAM_HPP_

#include <vector>

namespace nav2_drone_mpc_controller {

class Histogram {
public:
  /**
   * @brief Construct a Histogram with given bin count
   */
  explicit Histogram(size_t bins = 10);

  /**
   * @brief Add a value to the histogram
   */
  void addValue(double value);

  /**
   * @brief Retrieve the bin counts
   */
  const std::vector<size_t>& getBins() const;

private:
  size_t bins_;
  std::vector<size_t> counts_;
};

}  // namespace nav2_drone_mpc_controller

#endif  // NAV2_DRONE_MPC_CONTROLLER__HISTOGRAM_HPP_


// File: include/nav2_drone_mpc_controller/mpc_controller.hpp
#ifndef NAV2_DRONE_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define NAV2_DRONE_MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include <memory>
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_drone_mpc_controller {

class MPCController : public nav2_core::Controller {
public:
  /**
   * @brief Default constructor
   */
  MPCController();

  /**
   * @brief Configure the controller
   */
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 const std::string & plugin_name,
                 const std::shared_ptr<tf2_ros::Buffer> /*tf*/,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap*/) override;

  /**
   * @brief Compute velocity command given the current pose and velocity
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

  /**
   * @brief Check if goal is reached
   */
  bool isGoalReached(const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // MPC-specific parameters and state
};

}  // namespace nav2_drone_mpc_controller

#endif  // NAV2_DRONE_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
