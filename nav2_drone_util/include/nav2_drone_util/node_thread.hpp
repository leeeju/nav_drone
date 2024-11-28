#pragma once

#include <memory>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace nav2_drone_util
{

/**
 * @class nav_drone_util::NodeThread
 * @brief A utility class to process node or executor callbacks in a separate thread
 */
class NodeThread
{
public:
  /**
   * @brief Constructor for processing a node's callbacks in a background thread
   * @param node_base Interface to the node's base interface to spin in a separate thread
   */
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
  : node_(std::move(node_base)),
    thread_(std::make_unique<std::thread>(&NodeThread::spinNode, this))
  {}

  /**
   * @brief Constructor for processing executor callbacks in a background thread
   * @param executor Shared pointer to an executor to process callbacks
   */
  explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
  : executor_(std::move(executor)),
    thread_(std::make_unique<std::thread>(&NodeThread::spinExecutor, this))
  {}

  /**
   * @brief Constructor template for processing a node's callbacks
   * @param node A shared pointer to the node
   * @tparam NodeT The type of the node
   */
  template<typename NodeT>
  explicit NodeThread(std::shared_ptr<NodeT> node)
  : NodeThread(node->get_node_base_interface())
  {}

  /**
   * @brief Destructor to cleanly stop the thread
   */
  ~NodeThread()
  {
    if (thread_ && thread_->joinable()) {
      if (executor_) {
        executor_->cancel();  // Stop the executor gracefully
      }
      thread_->join();  // Join the thread to ensure proper cleanup
    }
  }

private:
  /**
   * @brief Spin the node in a background thread
   */
  void spinNode()
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    executor.spin();
    executor.remove_node(node_);
  }

  /**
   * @brief Spin the provided executor in a background thread
   */
  void spinExecutor()
  {
    if (executor_) {
      executor_->spin();
    }
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_{nullptr}; ///< Node base interface
  rclcpp::Executor::SharedPtr executor_{nullptr}; ///< Executor shared pointer
  std::unique_ptr<std::thread> thread_; ///< Thread for background spinning
};

}  // namespace nav_drone_util
