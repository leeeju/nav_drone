#pragma once

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace nav2_drone_util
{

/// @brief Measures execution time of code between calls to start() and end()
class ExecutionTimer
{
public:
  using Clock = std::chrono::high_resolution_clock;
  using nanoseconds = std::chrono::nanoseconds;
  using milliseconds = std::chrono::milliseconds;

  /// @brief Starts the timer. Call just prior to the code you want to measure.
  void start() { start_ = Clock::now(); is_running_ = true; }

  /// @brief Stops the timer. Call just after the code you want to measure.
  void end()
  {
    if (!is_running_)
    {
      throw std::logic_error("Timer must be started before calling end().");
    }
    end_ = Clock::now();
    is_running_ = false;
  }

  /// @brief Gets the elapsed time as a std::chrono::duration object in nanoseconds.
  /// @throws std::logic_error if end() is called before start().
  nanoseconds elapsed_time() const
  {
    ensure_timer_stopped();
    return end_ - start_;
  }

  /// @brief Gets the elapsed time as a floating-point number of seconds.
  /// @return Elapsed time in seconds.
  double elapsed_time_in_seconds() const
  {
    ensure_timer_stopped();
    return std::chrono::duration<double>(end_ - start_).count();
  }

  /// @brief Gets the elapsed time as a floating-point number of milliseconds.
  /// @return Elapsed time in milliseconds.
  double elapsed_time_in_milliseconds() const
  {
    ensure_timer_stopped();
    return std::chrono::duration<double, std::milli>(end_ - start_).count();
  }

  /// @brief Prints the elapsed time with a specified unit to the standard output.
  /// @param unit The time unit to display ("ns", "ms", or "s").
  void print_elapsed_time(const std::string &unit = "ms") const
  {
    ensure_timer_stopped();
    if (unit == "ns")
    {
      std::cout << "Elapsed time: " << elapsed_time().count() << " nanoseconds\n";
    }
    else if (unit == "ms")
    {
      std::cout << "Elapsed time: " << elapsed_time_in_milliseconds() << " milliseconds\n";
    }
    else if (unit == "s")
    {
      std::cout << "Elapsed time: " << elapsed_time_in_seconds() << " seconds\n";
    }
    else
    {
      throw std::invalid_argument("Invalid time unit. Use 'ns', 'ms', or 's'.");
    }
  }

private:
  /// @brief Ensures the timer has been started and stopped before querying elapsed time.
  void ensure_timer_stopped() const
  {
    if (is_running_)
    {
      throw std::logic_error("Timer is still running. Call end() before querying elapsed time.");
    }
  }

  Clock::time_point start_;
  Clock::time_point end_;
  bool is_running_ = false;
};

}  // namespace nav2_drone_util
