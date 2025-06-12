#ifndef NAV2_DRONE_CORE__BEHAVIOR_TREE_NAVIGATOR_HPP_
#define NAV2_DRONE_CORE__BEHAVIOR_TREE_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_drone_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"

namespace nav2_drone_core
{

/**
 * @struct FeedbackUtils
 * @brief Navigator feedback utilities required to get transforms, reference frames, and altitude information for drones.
 */
struct FeedbackUtils
{
  std::string drone_frame;
  std::string global_frame;
  double transform_tolerance;
  std::shared_ptr<tf2_ros::Buffer> tf;
};

class NavigatorMuxer
{
public:
  NavigatorMuxer() : current_navigator_(std::string("")) {}

  bool isNavigating()
  {
    std::lock_guard<std::mutex> l(mutex_);
    return !current_navigator_.empty();
  }

  void startNavigating(const std::string & navigator_name)
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (!current_navigator_.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMuxer"),
        "Another drone navigation task is already in progress.");
    }
    current_navigator_ = navigator_name;
  }

  void stopNavigating(const std::string & navigator_name)
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (current_navigator_ != navigator_name) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMuxer"),
        "Stopping incorrect drone navigation task.");
    } else {
      current_navigator_.clear();
    }
  }

protected:
  std::string current_navigator_;
  std::mutex mutex_;
};

class NavigatorBase
{
public:
  NavigatorBase() = default;
  virtual ~NavigatorBase() = default;

  virtual bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    const std::vector<std::string> & plugin_lib_names,
    const FeedbackUtils & feedback_utils,
    NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) = 0;

  virtual bool on_activate() = 0;
  virtual bool on_deactivate() = 0;
  virtual bool on_cleanup() = 0;
};


/**
 * @brief BehaviorTreeNavigator adapted for drones in 3D space
 */
template<class ActionT>
class BehaviorTreeNavigator : public NavigatorBase
{
public:
  using Ptr = std::shared_ptr<BehaviorTreeNavigator<ActionT>>;

  BehaviorTreeNavigator() : NavigatorBase(), plugin_muxer_(nullptr) {}
  virtual ~BehaviorTreeNavigator() = default;

  bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    const std::vector<std::string> & plugin_lib_names,
    const FeedbackUtils & feedback_utils,
    NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) final
  {
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    feedback_utils_ = feedback_utils;
    plugin_muxer_ = plugin_muxer;

    bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
      node,
      getName(),
      plugin_lib_names,
      getDefaultBTFilepath(parent_node),
      std::bind(&BehaviorTreeNavigator::onGoalReceived, this, std::placeholders::_1),
      std::bind(&BehaviorTreeNavigator::onLoop, this),
      std::bind(&BehaviorTreeNavigator::onPreempt, this, std::placeholders::_1),
      std::bind(&BehaviorTreeNavigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));

    bool ok = bt_action_server_->on_configure();
    bt_action_server_->getBlackboard()->set("tf_buffer", feedback_utils.tf);
    bt_action_server_->getBlackboard()->set("odom_smoother", odom_smoother);

    return configure(parent_node, odom_smoother) && ok;
  }

  bool on_activate() final { return bt_action_server_->on_activate() && activate(); }
  bool on_deactivate() final { return bt_action_server_->on_deactivate() && deactivate(); }
  bool on_cleanup() final
  {
    bt_action_server_->on_cleanup();
    bt_action_server_.reset();
    return cleanup();
  }

  virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;
  virtual std::string getName() = 0;

protected:
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;
  virtual void onLoop() = 0;
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;
  virtual void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) = 0;

  virtual bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr, std::shared_ptr<nav2_util::OdomSmoother>) { return true; }
  virtual bool cleanup() { return true; }
  virtual bool activate() { return true; }
  virtual bool deactivate() { return true; }

  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("DroneNavigator")};
  rclcpp::Clock::SharedPtr clock_;
  FeedbackUtils feedback_utils_;
  NavigatorMuxer * plugin_muxer_;
};

} // namespace nav2_drone_core

#endif  // NAV2_DRONE_CORE__BEHAVIOR_TREE_NAVIGATOR_HPP_
