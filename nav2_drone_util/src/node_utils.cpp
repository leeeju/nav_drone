#include "nav2_drone_util/node_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

namespace nav2_drone_util {

// Example utility function to print all parameters of a node
void print_all_parameters(rclcpp::Node::SharedPtr node)
{
  auto parameters = node->list_parameters({}, 10);  // Retrieve parameters from the node

  RCLCPP_INFO(node->get_logger(), "Listing parameters for node: %s", node->get_name());
  for (const auto & name : parameters.names) {
    try {
      auto param = node->get_parameter(name);
      RCLCPP_INFO(node->get_logger(), "Parameter: %s, Value: %s", name.c_str(), param.value_to_string().c_str());
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
      RCLCPP_WARN(node->get_logger(), "Parameter %s not declared, skipping.", name.c_str());
    }
  }
}

// Example utility function to safely declare and set a parameter
void declare_and_set_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string & param_name,
  const rclcpp::ParameterValue & value,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor = rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    RCLCPP_INFO(node->get_logger(), "Declaring parameter: %s", param_name.c_str());
    node->declare_parameter(param_name, value, descriptor);
  } else {
    RCLCPP_INFO(node->get_logger(), "Parameter %s already declared, setting value.", param_name.c_str());
  }

  node->set_parameter(rclcpp::Parameter(param_name, value));
}

// Example utility function to retrieve a parameter with a fallback
template<typename T>
T get_parameter_with_default(
  rclcpp::Node::SharedPtr node,
  const std::string & param_name,
  const T & default_value)
{
  T value;
  if (!node->get_parameter(param_name, value)) {
    RCLCPP_WARN(node->get_logger(), "Parameter %s not set, using default: %s",
      param_name.c_str(), std::to_string(default_value).c_str());
    value = default_value;
  }
  return value;
}

}  // namespace nav_drone_util
