#include "nav2_drone_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_drone_util {

// Define the implementation of the utility functions if needed

// Explicit template instantiation for the declare_parameter_if_not_declared function
template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor)
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
    RCLCPP_INFO(node->get_logger(), "Parameter '%s' declared with default value.", param_name.c_str());
  }
}

// Explicit template instantiation for the get_plugin_type_param function
template<typename NodeT>
std::string get_plugin_type_param(
  NodeT node,
  const std::string & plugin_name)
{
  declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::ParameterValue(""),
    rcl_interfaces::msg::ParameterDescriptor().set_description("Plugin type for " + plugin_name));

  std::string plugin_type;
  if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to get 'plugin' parameter for plugin: %s", plugin_name.c_str());
    throw std::runtime_error("Plugin parameter not defined: " + plugin_name);
  }

  RCLCPP_INFO(node->get_logger(), "Plugin type for '%s': %s", plugin_name.c_str(), plugin_type.c_str());
  return plugin_type;
}

// Additional utility functions can be implemented here as needed

} // namespace nav2_drone_util
