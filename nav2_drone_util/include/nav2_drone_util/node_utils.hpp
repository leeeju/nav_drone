#ifndef NAV2_DRONE_UTIL__NODE_UTILS_HPP_
#define NAV2_DRONE_UTIL__NODE_UTILS_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace nav2_drone_util
{
/**
 * @brief Declare a parameter on the node if not already declared, with explicit descriptor.
 *
 * @tparam NodeT Node pointer or shared_ptr type supporting declare_parameter
 * @param node Node instance to declare parameter on
 * @param name Name of the parameter
 * @param default_value Default value to declare
 * @param descriptor Parameter descriptor for additional metadata
 */
template<class NodeT>
inline void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_value, descriptor);
  }
}

/**
 * @brief Declare a parameter on the node if not already declared, using default descriptor.
 *
 * @tparam NodeT Node pointer or shared_ptr type supporting declare_parameter
 * @param node Node instance to declare parameter on
 * @param name Name of the parameter
 * @param default_value Default value to declare
 */
template<class NodeT>
inline void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & name,
  const rclcpp::ParameterValue & default_value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  declare_parameter_if_not_declared(node, name, default_value, descriptor);
}

/**
 * @brief Retrieve a string parameter for plugin type from the node.
 *
 * @tparam NodeT Node pointer or shared_ptr type supporting get_parameter
 * @param node Node instance to query parameter
 * @param name Name of the parameter
 * @return The parameter value as string
 */
template<class NodeT>
inline std::string get_plugin_type_param(
  NodeT node,
  const std::string & name)
{
  std::string plugin_type;
  node->get_parameter(name, plugin_type);
  return plugin_type;
}

}  // namespace nav2_drone_util

#endif  // NAV2_DRONE_UTIL__NODE_UTILS_HPP_
