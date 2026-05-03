#ifndef CHRONOMETER_HPP
#define CHRONOMETER_HPP

#include <optional>
#include <rclcpp/rclcpp.hpp>

/**
 * @class Chronometer
 * @brief Utility for non-blocking timing in ROS 2 State Machines.
 * 
 * Provides a lightweight mechanism to track elapsed time without halting the 
 * node's execution thread. This is essential for implementing timeouts and 
 * temporal transitions within a Finite State Machine (FSM).
 */
class Chronometer {
public:
  /**
   * @brief Constructs a chronometer linked to a specific clock.
   * @param[in] clock Shared pointer to the node's clock (e.g., node->get_clock()).
   */
  explicit Chronometer(rclcpp::Clock::SharedPtr clock) :
    clock_(clock), timeout_duration_(0.0) {}

  /**
   * @brief Arms the timer with a specific duration.
   * 
   * Captures the current timestamp and defines the future expiration point.
   * 
   * @param[in] seconds Duration in seconds until the timeout is reached.
   */
  void start(double seconds) {
    timeout_duration_ = seconds;
    start_time_ = clock_->now();
  }

  /**
   * @brief Evaluates if the programmed time has elapsed.
   * 
   * Compares the current clock time against the captured start time.
   * 
   * @return true if elapsed time >= programmed duration, false otherwise.
   */
  bool timeout_reached() const {
    if (!start_time_.has_value()) return false;
    return get_elapsed_seconds() >= timeout_duration_;
  }

  /** 
   * @brief Clears the internal timestamp and resets the duration. 
   * Useful for stopping the timer or preparing it for a new cycle.
   */
  void reset() {
    start_time_.reset();
    timeout_duration_ = 0.0;
  }

  /** @return true if the chronometer has been started and not yet reset. */
  bool is_active() const { return start_time_.has_value(); }

private:
  /**
   * @brief Calculates the delta between now and the start timestamp.
   * @return Elapsed time in seconds.
   */
  double get_elapsed_seconds() const {
    if (!start_time_.has_value()) return 0.0;

    // ROS 2 Duration handles the conversion between nanoseconds and seconds
    rclcpp::Duration diff = clock_->now() - start_time_.value();
    return diff.seconds();
  }

  /// Reference to the system or ROS clock.
  rclcpp::Clock::SharedPtr clock_;
  
  /// Target duration for the current timing operation.
  double timeout_duration_;
  
  /// Captured timestamp when start() was invoked.
  std::optional<rclcpp::Time> start_time_;
};

#endif // CHRONOMETER_HPP