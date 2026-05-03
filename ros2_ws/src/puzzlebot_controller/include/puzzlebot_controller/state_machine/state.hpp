#ifndef STATE_HPP
#define STATE_HPP

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "puzzlebot_controller/controller/puzzlebot_controller.hpp"
#include "puzzlebot_controller/utils/robot_context.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class State
 * @brief Abstract base class for the Finite State Machine (FSM).
 * 
 * Implements the 'State' pattern to encapsulate behavior associated with 
 * particular robot modes (e.g., LineFollowing, ObstacleAvoidance). 
 * Provides automated lifecycle management for entry, execution, and exit phases.
 */
class State {
public:
  /**
   * @param alias Unique string identifier for the state (used for logging and transitions).
   */
  explicit State(std::string alias) : alias_(std::move(alias)), is_first_run_(true) {}

  virtual ~State() = default;

  /** @return The human-readable identifier of the state. */
  std::string get_alias() const { return alias_; }

  /**
   * @brief Dependency injection for the state's required tools and shared data.
   * 
   * @param robot_ctrl Shared pointer to the actuator/controller interface.
   * @param logger ROS 2 logger instance for state-specific reporting.
   * @param clock ROS 2 clock for timing and duration-based logic.
   * @param ctx Shared context containing dynamic sensor data and global flags.
   */
  void setup(std::shared_ptr<PuzzlebotController> robot_ctrl, rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock, std::shared_ptr<RobotContext> ctx) {
    robot_ctrl_ = robot_ctrl;
    logger_ = logger;
    clock_ = clock;
    ctx_ = ctx;
  }

  /**
   * @brief Manages the high-level execution flow.
   * 
   * Automatically triggers on_entry() during the first call, followed by on_execute().
   * 
   * @return std::optional<std::string> An event string to trigger a transition to a 
   * new state, or std::nullopt to continue execution in the current state.
   */
  std::optional<std::string> execute() {
    if (is_first_run_) {
      RCLCPP_INFO(logger_, "Entered State: [%s]", alias_.c_str());
      on_entry();
      is_first_run_ = false;
    }
    return on_execute();
  }

  /**
   * @brief Resets the lifecycle flag and triggers exit cleanup.
   * Should be called by the FSM manager when transitioning AWAY from this state.
   */
  void reset_lifecycle() {
    on_exit();
    is_first_run_ = true;
  }

protected:
  /** @brief Logic to execute once when the state becomes active. */
  virtual void on_entry() {}

  /** 
   * @brief Core logic executed on every control loop iteration. 
   * @return Event trigger for transition or std::nullopt.
   */
  virtual std::optional<std::string> on_execute() { return std::nullopt; }

  /** @brief Cleanup logic to execute before transitioning to a different state. */
  virtual void on_exit() {}

  /// State identifier.
  std::string alias_;

  /// Interface to robot actuators and low-level controllers.
  std::shared_ptr<PuzzlebotController> robot_ctrl_;

  /// Logger handle for system diagnostics.
  rclcpp::Logger logger_ = rclcpp::get_logger("state_machine");
  
  /// Shared clock for time-stamping and timeouts.
  rclcpp::Clock::SharedPtr clock_;
  
  /// Shared robot context (Blackboard pattern) for inter-state data sharing.
  std::shared_ptr<RobotContext> ctx_;

private:
  /// Flag to ensure on_entry() is only called once per activation.
  bool is_first_run_;
};

#endif // STATE_HPP