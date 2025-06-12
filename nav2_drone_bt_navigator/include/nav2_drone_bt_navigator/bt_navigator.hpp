// File: include/nav2_drone_bt_navigator/drone_bt_navigator.hpp
#ifndef NAV2_DRONE_BT_NAVIGATOR__DRONE_BT_NAVIGATOR_HPP_
#define NAV2_DRONE_BT_NAVIGATOR__DRONE_BT_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/odometry_utils.hpp"

#include "nav2_drone_core/planner.hpp"
#include "nav2_drone_core/controller.hpp"
#include "nav2_drone_core/goal_checker.hpp"
#include "nav2_drone_core/progress_checker.hpp"
#include "nav2_drone_core/behavior_tree_navigator.hpp"

namespace nav2_drone_bt_navigator
{

/**
 * @class nav2_drone_bt_navigator::DroneBtNavigator
 * @brief A lifecycle node that uses BehaviorTree to navigate drones in 3D space.
 */
class DroneBtNavigator : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit DroneBtNavigator(rclcpp::NodeOptions options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~DroneBtNavigator();

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Plugin loader for drone-specific navigator plugins
  pluginlib::ClassLoader<nav2_core::NavigatorBase> class_loader_;
  std::vector<pluginlib::UniquePtr<nav2_core::NavigatorBase>> navigators_;
  nav2_core::NavigatorMuxer plugin_muxer_;

  // Odometry smoother for drones
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  // Frame and topic settings
  std::string global_frame_;
  std::string drone_frame_;
  double transform_tolerance_;
  double filter_duration_;
  std::string odom_topic_;

  // TF listener for 3D transforms
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace nav2_drone_bt_navigator

#endif  // NAV2_DRONE_BT_NAVIGATOR__DRONE_BT_NAVIGATOR_HPP_
