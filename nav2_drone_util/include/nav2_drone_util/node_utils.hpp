#ifndef NAV2_DRONE_UTIL__NODE_UTILS_HPP_
#define NAV2_DRONE_UTIL__NODE_UTILS_HPP_

#include <string>
#include <cmath>
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

/**
 * @brief Convert radians to degrees.
 *
 * @param rad Angle in radians
 * @return Angle in degrees
 */
inline double rad_to_deg(double rad)
{
  return rad * (180.0 / M_PI);
}

/**
 * @brief Convert degrees to radians.
 *
 * @param deg Angle in degrees
 * @return Angle in radians
 */
inline double deg_to_rad(double deg)
{
  return deg * (M_PI / 180.0);
}

inline double getDiff2Angles(double angle1, double angle2)
{
  double diff = angle2 - angle1;
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  return diff;
}

}  // namespace nav2_drone_util

#endif  // NAV2_DRONE_UTIL__NODE_UTILS_HPP_
