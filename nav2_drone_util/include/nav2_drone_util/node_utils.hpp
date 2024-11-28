#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include <stdexcept>

namespace nav2_drone_util {

/**
 * @brief Declares a parameter if it hasn't been declared yet and initializes it with a default value.
 *
 * @param node The node where the parameter is declared.
 * @param param_name Name of the parameter to declare.
 * @param default_value Default value to initialize the parameter if it hasn't been declared.
 * @param parameter_descriptor Optional descriptor for the parameter.
 */
template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor = {})
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

/**
 * @brief Declares a parameter with a specific type if it hasn't been declared yet.
 *
 * @param node The node where the parameter is declared.
 * @param param_name Name of the parameter to declare.
 * @param param_type The type of the parameter (e.g., rclcpp::PARAMETER_INTEGER).
 * @param parameter_descriptor Optional descriptor for the parameter.
 */
template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterType & param_type,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor = {})
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, param_type, parameter_descriptor);
  }
}

/**
 * @brief Retrieves the type of a plugin specified by its parameter.
 *
 * This function fetches the value of the parameter "<plugin_name>.plugin" and ensures the parameter is declared.
 *
 * @param node The node from which the plugin type is retrieved.
 * @param plugin_name Name of the plugin whose type parameter is being retrieved.
 * @return A string representing the plugin type.
 * @throws std::runtime_error If the parameter cannot be retrieved.
 */
template<typename NodeT>
std::string get_plugin_type_param(
  NodeT node,
  const std::string & plugin_name)
{
  declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::ParameterValue(""));

  std::string plugin_type;
  try {
    if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
      throw std::runtime_error(
        "Failed to retrieve plugin type for " + plugin_name + ". Check parameter definition.");
    }
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
    RCLCPP_FATAL(
      node->get_logger(),
      "Plugin parameter '%s.plugin' not declared for %s. Exception: %s",
      plugin_name.c_str(), plugin_name.c_str(), ex.what());
    throw;
  } catch (const rclcpp::exceptions::ParameterImmutableException & ex) {
    RCLCPP_FATAL(
      node->get_logger(),
      "Plugin parameter '%s.plugin' is immutable for %s. Exception: %s",
      plugin_name.c_str(), plugin_name.c_str(), ex.what());
    throw;
  }
  return plugin_type;
}

}  // namespace nav2_drone_util
