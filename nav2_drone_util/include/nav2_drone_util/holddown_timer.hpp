#pragma once

#include <chrono>

namespace nav2_drone_util {

/**
 * @brief A utility timer that triggers a "hold-down" period once a condition is met.
 *
 * The timer starts when the given condition becomes true and remains active
 * for a specified duration. It resets if the condition fails during this period.
 */
class HolddownTimer {
public:
  using Clock = std::chrono::high_resolution_clock;
  using Seconds = std::chrono::duration<double>;

  /**
   * @brief Constructor to initialize the timer with a specific hold-down duration
   * @param duration_seconds The duration for the hold-down period in seconds
   */
  explicit HolddownTimer(double duration_seconds)
  : duration_{Seconds(duration_seconds)}, running_{false} {}

  /**
   * @brief Test and manage the hold-down timer based on a condition
   *
   * If the condition is true, the timer starts and checks if the hold-down duration has passed.
   * If the condition becomes false, the timer resets.
   *
   * @param condition The condition to evaluate for triggering or maintaining the timer
   * @return True if the timer is running and the hold-down period has elapsed; otherwise, false
   */
  bool test(bool condition) {
    auto current_time = Clock::now();

    if (!running_ && condition) {
      // Start the timer when the condition is first met
      running_ = true;
      start_time_ = current_time;
    } else if (running_ && !condition) {
      // Reset the timer if the condition fails
      running_ = false;
    }

    if (running_) {
      // Check if the hold-down duration has passed
      Seconds elapsed = current_time - start_time_;
      return elapsed >= duration_;
    }

    return false;
  }

  /**
   * @brief Reset the timer manually
   */
  void reset() {
    running_ = false;
  }

  /**
   * @brief Adjust the hold-down duration dynamically
   * @param duration_seconds The new hold-down duration in seconds
   */
  void setDuration(double duration_seconds) {
    duration_ = Seconds(duration_seconds);
  }

  /**
   * @brief Get the current hold-down duration
   * @return The hold-down duration in seconds
   */
  double getDuration() const {
    return duration_.count();
  }

private:
  Seconds duration_;                        ///< Hold-down period duration
  Clock::time_point start_time_;            ///< Time point when the timer started
  bool running_;                            ///< Indicates if the timer is currently active
};

} // namespace nav_drone_util
