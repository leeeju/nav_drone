#include <memory>
#include <utility>  // For std::move
#include "nav2_drone_util/node_thread.hpp"

namespace nav2_drone_util
{

NodeThread::NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
: node_(std::move(node_base))  // Use move semantics for efficiency
{
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Start a background thread to manage the executor and node callbacks
  thread_ = std::make_unique<std::thread>(
    [this]() {
      try {
        executor_->add_node(node_);
        executor_->spin();
        executor_->remove_node(node_);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          rclcpp::get_logger("nav_drone_util"),
          "Exception in NodeThread executor: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(
          rclcpp::get_logger("nav_drone_util"),
          "Unknown exception in NodeThread executor.");
      }
    });
}

NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
: executor_(std::move(executor))  // Use move semantics for efficiency
{
  // Start a background thread to spin the provided executor
  thread_ = std::make_unique<std::thread>(
    [this]() {
      try {
        executor_->spin();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          rclcpp::get_logger("nav_drone_util"),
          "Exception in NodeThread executor spin: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(
          rclcpp::get_logger("nav_drone_util"),
          "Unknown exception in NodeThread executor spin.");
      }
    });
}

NodeThread::~NodeThread()
{
  // Safely stop the executor and join the thread
  if (executor_) {
    executor_->cancel();
  }
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
}

}  // namespace nav2_drone_util
