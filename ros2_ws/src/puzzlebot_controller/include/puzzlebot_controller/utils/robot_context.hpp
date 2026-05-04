#ifndef ROBOT_CONTEXT_HPP
#define ROBOT_CONTEXT_HPP

#include "puzzlebot_interfaces/msg/traffic_sign_detection.hpp"
#include <mutex>
#include <optional>

/**
 * @struct RobotContext
 * @brief Shared data container (Blackboard) for the Puzzlebot state machine.
 *
 * This structure acts as the robot's "short-term memory", storing the most
 * recent environmental detections. It decouples the asynchronous computer
 * vision callbacks from the synchronous state machine execution loop.
 */
struct RobotContext {
  using TrafficSign = puzzlebot_interfaces::msg::TrafficSignDetection;

  /**
   * @brief Directional traffic signs (e.g., Mandatory Turn Arrows).
   */
  std::optional<TrafficSign> direction;

  /**
   * @brief Traffic light status (Red, Yellow, Green).
   */
  std::optional<TrafficSign> tf_light;

  /**
   * @brief Regulatory signs (e.g., Stop, Give Way, Speed Limits).
   */
  std::optional<TrafficSign> regulatory;

  /**
   * @brief Caution and Environmental signs (e.g., Hazards, Roadwork).
   */
  std::optional<TrafficSign> caution;

  /**
   * @brief Thread-safety mutex.
   * Protects context data when accessed by multiple ROS 2 executors or threads.
   */
  mutable std::mutex memory_mutex;

  /**
   * @brief Flag indicating if the line/path has been successfully re-detected.
   *
   * This flag is set to 'false' upon reaching a Street Crossing event,
   * effectively arming the state machine for an open-loop maneuver.
   * It switches to 'true' once the path_interpreter confirms a stable
   * detection.
   */
  bool is_path_recovered;

  /**
   * @brief Current count of consecutive valid path detections.
   * Used to filter transient noise and ensure a stable transition
   * after open-loop maneuvers (e.g., Street Crossings).
   */
  int recovery_hits = 0;

  /**
   * @brief Threshold for path validation.
   * Requires 10 consecutive hits (~333ms @ 30FPS) to confirm
   * path recovery and switch back to PID control.
   */
  const int HIT_THRESHOLD = 30;

  /**
   * @brief Flushes all stored detections.
   * Call this when a maneuver is completed to prevent "stale" detections
   * from triggering incorrect transitions in the next state.
   */
  void clear_all() {
    std::lock_guard<std::mutex> lock(memory_mutex);
    direction.reset();
    tf_light.reset();
    regulatory.reset();
    caution.reset();
  }

  /**
   * @brief Checks if the blackboard is currently empty.
   * @return true if no signs are currently stored.
   */
  bool is_empty() const {
    std::lock_guard<std::mutex> lock(memory_mutex);
    return !direction && !tf_light && !regulatory && !caution;
  }
};

#endif // ROBOT_CONTEXT_HPP