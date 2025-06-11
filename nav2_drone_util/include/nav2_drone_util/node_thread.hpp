#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include "rclcpp/rclcpp.hpp"

namespace nav2_drone_util
{

/**
 * @class nav2_drone_util::NodeThread
 * @brief A utility class that spins a node or executor in a separate thread.
 *
 * Useful for handling background tasks such as processing callbacks or managing
 * event loops without blocking the main thread.
 */
class NodeThread
{
public:
  /**
   * @brief Construct a NodeThread to spin a node's callbacks in a background thread.
   * @param node_base The base interface of the node to spin in a thread.
   */
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base);

  /**
   * @brief Construct a NodeThread to spin an executor's callbacks in a background thread.
   * @param executor A shared pointer to the executor to spin in the thread.
   */
  explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

  /**
   * @brief Construct a NodeThread to spin a templated node in a background thread.
   * @param node A pointer to a templated node to spin in the thread.
   */
  template<typename NodeT>
  explicit NodeThread(NodeT node)
  : NodeThread(node->get_node_base_interface())
  {}

  /**
   * @brief Destructor to ensure proper cleanup of the thread and executor.
   */
  ~NodeThread();

  /**
   * @brief Check if the thread is currently running.
   * @return True if the thread is running, false otherwise.
   */
  bool is_running() const;

  /**
   * @brief Stop the spinning thread.
   */
  void stop();

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;  ///< Node interface to spin
  std::unique_ptr<std::thread> thread_;                        ///< Background thread
  rclcpp::Executor::SharedPtr executor_;                       ///< Executor managing the callbacks
  std::atomic<bool> running_{false};                           ///< Indicates if the thread is running
};

}  // namespace nav2_drone_util
