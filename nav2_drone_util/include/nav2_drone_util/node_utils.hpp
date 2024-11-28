#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions/exceptions.hpp"

namespace nav2_drone_util {

/**
 * @brief Declares a static ROS2 parameter and sets it to a default value if not already declared
 *
 * This utility function ensures that the parameter is declared in the node, avoiding runtime
 * errors caused by attempting to access undeclared parameters.
 *
 * @tparam NodeT The node type
 * @param node The node in which the parameter is to be declared
 * @param param_name The name of the parameter
 * @param default_value The default value to initialize the parameter with
 * @param parameter_descriptor (Optional) The parameter descriptor for additional metadata
 */
template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

/**
 * @brief Declares a static ROS2 parameter with a given type if not already declared
 *
 * This function is useful when the parameter type is known and explicitly specified.
 *
 * @tparam NodeT The node type
 * @param node The node in which the parameter is to be declared
 * @param param_name The name of the parameter
 * @param param_type The type of the parameter
 * @param parameter_descriptor (Optional) The parameter descriptor for additional metadata
 */
template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterType & param_type,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, param_type, parameter_descriptor);
  }
}

/**
 * @brief Retrieves the type of plugin associated with a given plugin name
 *
 * This utility function searches for a parameter `<plugin_name>.plugin` in the node
 * and retrieves its value. It ensures the parameter is declared before attempting to access it.
 *
 * @tparam NodeT The node type
 * @param node The node from which the plugin type parameter will be retrieved
 * @param plugin_name The name of the plugin
 * @return A string containing the type of the plugin
 */
template<typename NodeT>
std::string get_plugin_type_param(
  NodeT node,
  const std::string & plugin_name)
{
  // Declare the plugin parameter if not already declared
  declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::PARAMETER_STRING);

  // Retrieve the plugin type parameter
  std::string plugin_type;
  try {
    if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
      RCLCPP_FATAL(
        node->get_logger(), "Failed to retrieve 'plugin' parameter for %s", plugin_name.c_str());
      exit(EXIT_FAILURE);
    }
  } catch (const rclcpp::exceptions::ParameterImmutableException & ex) {
    RCLCPP_FATAL(
      node->get_logger(),
      "Parameter 'plugin' for %s is immutable or not defined: %s",
      plugin_name.c_str(), ex.what());
    exit(EXIT_FAILURE);
  }

  return plugin_type;
}

} // namespace nav_drone_util
