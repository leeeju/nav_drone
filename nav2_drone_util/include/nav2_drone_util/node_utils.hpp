#ifndef NAV2_DRONE_UTIL__NODE_UTILS_HPP_
#define NAV2_DRONE_UTIL__NODE_UTILS_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace nav2_drone_util
{

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
