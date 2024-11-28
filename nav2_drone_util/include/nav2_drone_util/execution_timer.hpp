#pragma once

#include <chrono>
#include <string>
#include <iostream>

namespace nav2_drone_util
{

/**
 * @brief Utility class for measuring execution time between start and stop calls.
 * It provides elapsed time in both nanoseconds and seconds, with optional logging.
 */
class ExecutionTimer
{
public:
  using Clock = std::chrono::high_resolution_clock;
  using nanoseconds = std::chrono::nanoseconds;

  /**
   * @brief Starts the timer. Call this just before the code you want to measure.
   */
  void start()
  {
    start_ = Clock::now();
    running_ = true;
  }

  /**
   * @brief Stops the timer. Call this just after the code you want to measure.
   */
  void stop()
  {
    if (!running_)
    {
      throw std::logic_error("Timer was not started before calling stop().");
    }
    end_ = Clock::now();
    running_ = false;
  }

  /**
   * @brief Resets the timer, clearing start and end times.
   */
  void reset()
  {
    running_ = false;
    start_ = Clock::time_point();
    end_ = Clock::time_point();
  }

  /**
   * @brief Gets the elapsed time as an integral std::chrono::duration object.
   * @return Elapsed time in nanoseconds.
   */
  nanoseconds elapsed_time() const
  {
    if (running_)
    {
      throw std::logic_error("Timer is still running. Call stop() before querying elapsed time.");
    }
    return end_ - start_;
  }

  /**
   * @brief Gets the elapsed time as a floating-point number of seconds.
   * @return Elapsed time in seconds.
   */
  double elapsed_time_in_seconds() const
  {
    if (running_)
    {
      throw std::logic_error("Timer is still running. Call stop() before querying elapsed time.");
    }
    return std::chrono::duration<double>(end_ - start_).count();
  }

  /**
   * @brief Logs the elapsed time to the console in seconds with an optional label.
   * @param label A string label for the log (optional).
   */
  void log_elapsed_time(const std::string &label = "") const
  {
    if (running_)
    {
      throw std::logic_error("Timer is still running. Call stop() before logging elapsed time.");
    }
    if (!label.empty())
    {
      std::cout << label << ": ";
    }
    std::cout << "Elapsed time: " << elapsed_time_in_seconds() << " seconds.\n";
  }

private:
  Clock::time_point start_{};
  Clock::time_point end_{};
  bool running_{false}; ///< Indicates if the timer is currently running.
};

}  // namespace nav2_drone_util
