#pragma once

#include <stdexcept>
#include <string>
#include <memory>

namespace nav2_drone_core
{

class DroneException : public std::runtime_error
{
public:
  explicit DroneException(const std::string description)
  : std::runtime_error(description) {}
  using Ptr = std::shared_ptr<DroneException>;
};



}  // namespace nav2_drone_core
