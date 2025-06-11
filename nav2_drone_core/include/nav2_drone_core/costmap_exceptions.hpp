#pragma once

#include <stdexcept>
#include <string>

namespace nav2_drone_core
{

class CostmapException : public std::runtime_error
{
public:
  explicit CostmapException(const std::string & description)
  : std::runtime_error(description) {}
};

class CostmapTFError : public CostmapException
{
public:
  explicit CostmapTFError(const std::string & description)
  : CostmapException(description) {}
};

class InvalidResolutionError : public CostmapException
{
public:
  explicit InvalidResolutionError(const std::string & description)
  : CostmapException(description) {}
};
class CostmapUpdateError : public CostmapException
{
public:
  explicit CostmapUpdateError(const std::string & description)
  : CostmapException(description) {}
};


} // namespace nav2_drone_core
