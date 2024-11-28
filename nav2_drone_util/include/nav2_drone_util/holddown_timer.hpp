#pragma once

#include <chrono>  // For high_resolution_clock

namespace nav2_drone_util {

/**
 * @brief A timer that activates when a condition holds true for a specified duration.
 */
class HolddownTimer {
public:
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;

  /**
   * @brief Construct a HolddownTimer.
   * @param duration_sec Duration in seconds that the condition must hold true.
   */
  explicit HolddownTimer(double duration_sec)
  : duration_{std::chrono::duration<double>(duration_sec)}, running_{false} {}

  /**
   * @brief Test the condition and update the timer state.
   *
   * Starts the timer when the condition is true. If the condition remains true
   * for the specified duration, the function returns true. The timer resets when
   * the condition becomes false.
   *
   * @param condition The condition to evaluate.
   * @return True if the condition has held for the specified duration, false otherwise.
   */
  bool test(bool condition) {
    auto current_time = Clock::now();

    if (!running_ && condition) {
      // Start the timer when the condition becomes true
      running_ = true;
      start_time_ = current_time;
    } else if (!condition) {
      // Reset the timer if the condition becomes false
      running_ = false;
    }

    if (running_) {
      auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time_);
      return elapsed >= duration_;
    }

    return false;
  }

  /**
   * @brief Resets the timer to its initial state.
   */
  void reset() {
    running_ = false;
  }

  /**
   * @brief Check if the timer is currently running.
   * @return True if the timer is active, false otherwise.
   */
  bool is_running() const {
    return running_;
  }

private:
  std::chrono::duration<double> duration_;  // Duration the condition must hold
  TimePoint start_time_;                    // Start time of the timer
  bool running_;                            // Whether the timer is currently active
};

}  // namespace nav2_drone_util
