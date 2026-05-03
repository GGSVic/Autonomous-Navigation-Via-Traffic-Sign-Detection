#ifndef FINITE_STATE_MACHINE_HPP
#define FINITE_STATE_MACHINE_HPP

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "puzzlebot_controller/controller/puzzlebot_controller.hpp"
#include "puzzlebot_controller/utils/robot_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state.hpp"

/**
 * @class FiniteStateMachine
 * @brief Orchestrator node that manages state execution and event-based transitions.
 *
 * Implements a reactive FSM using unordered maps for O(1) state lookups.
 */
class FiniteStateMachine : public rclcpp::Node {
public:
  /**
   * @brief Transition table for a specific state.
   * Key: Triggering event (e.g., "stop_sign_detected").
   * Value: Alias of the target destination state.
   */
  using TransitionTable = std::unordered_map<std::string, std::string>;

  /**
   * @brief FSM Constructor.
   * @param fsm_name Name of the node in the ROS 2 graph.
   * @param robot_ctrl Pointer to the motion and hardware interface.
   * @param ctx Pointer to the robot's short-term memory and sensor context.
   * @param loop_rate_s Execution frequency in seconds.
   */
  FiniteStateMachine(const std::string& fsm_name,
    std::shared_ptr<PuzzlebotController> robot_ctrl, std::shared_ptr<RobotContext> ctx,
    double loop_rate_s);

  /**
   * @brief Registers a state and injects all required dependencies.
   */
  void add_state(std::shared_ptr<State> state, TransitionTable transitions);

  /** @brief Enables the control loop execution. */
  void start();

  /**
   * @brief Manages transitions based on an event trigger.
   */
  void handle_event(const std::string& event);

private:
  /** @brief Core control loop. Executes the active state logic. */
  void control_callback();

  // ROS 2 Infrastructure
  rclcpp::TimerBase::SharedPtr timer_;

  /** @brief The central interface for robot movement and control. */
  std::shared_ptr<PuzzlebotController> robot_ctrl_;

  std::shared_ptr<RobotContext> ctx_;

  // FSM Logic Containers - Using unordered_map for performance
  std::unordered_map<std::string, std::shared_ptr<State>> states_registry_;
  std::unordered_map<std::string, TransitionTable> state_transitions_;

  std::string current_state_alias_ = "";
  std::string last_event_ = "";
  bool blocked_ = true;
};

#endif // FINITE_STATE_MACHINE_HPP