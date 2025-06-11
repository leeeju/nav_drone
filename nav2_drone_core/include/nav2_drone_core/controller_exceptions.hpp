#ifndef nav2_drone_core__CONTROLLER_EXCEPTIONS_HPP_
#define nav2_drone_core__CONTROLLER_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace nav2_drone_core
{

class ControllerException : public std::runtime_error
{
public:
  explicit ControllerException(const std::string & description)
  : std::runtime_error(description) {}
};

class ControllerTFError : public ControllerException
{
public:
  explicit ControllerTFError(const std::string & description)
  : ControllerException(description) {}
};

class FailedToMakeProgress : public ControllerException
{
public:
  explicit FailedToMakeProgress(const std::string & description)
  : ControllerException(description) {}
};

class PatienceExceeded : public ControllerException
{
public:
  explicit PatienceExceeded(const std::string & description)
  : ControllerException(description) {}
};

class InvalidPath : public ControllerException
{
public:
  explicit InvalidPath(const std::string & description)
  : ControllerException(description) {}
};

class NoValidControl : public ControllerException
{
public:
  explicit NoValidControl(const std::string & description)
  : ControllerException(description) {}
};

class NoValidWaypoint : public ControllerException
{
public:
  explicit NoValidWaypoint(const std::string & description)
  : ControllerException(description) {}
};

class InvalidController : public ControllerException
{
public:
  explicit InvalidController(const std::string & description)
  : ControllerException(description) {}
};

}  // namespace nav2_drone_core

#endif  // nav2_drone_core__CONTROLLER_EXCEPTIONS_HPP_
