#include <memory>
#include <stdexcept>
#include "nav2_drone_util/node_thread.hpp"

namespace nav2_drone_util
{

NodeThread::NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
: node_(node_base)
{
  // Initialize a single-threaded executor
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Start a thread to manage the executor and process callbacks
  running_ = true;
  thread_ = std::make_unique<std::thread>(
    [&]()
    {
      try {
        executor_->add_node(node_);
        executor_->spin();
        executor_->remove_node(node_);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("nav2_drone_util"),
          "Exception in NodeThread executor: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("nav2_drone_util"),
          "Unknown exception in NodeThread executor.");
      }
      running_ = false;
    });
}

NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
: executor_(executor)
{
  // Start a thread to spin the provided executor
  running_ = true;
  thread_ = std::make_unique<std::thread>(
    [&]()
    {
      try {
        executor_->spin();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("nav2_drone_util"),
          "Exception in NodeThread executor spin: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("nav2_drone_util"),
          "Unknown exception in NodeThread executor spin.");
      }
      running_ = false;
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
  running_ = false;
}

bool NodeThread::is_running() const
{
  return running_;
}

void NodeThread::stop()
{
  if (executor_) {
    executor_->cancel();
  }
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
  running_ = false;
}

}  // namespace nav2_drone_util
