#include "puzzlebot_controller/state_machine/state_machine.hpp"

/**
 * @brief Constructor for the Finite State Machine manager.
 * Initializes the cyclic timer that drives the behavior of the active state.
 */
FiniteStateMachine::FiniteStateMachine(const std::string& fsm_name,
  std::shared_ptr<PuzzlebotController> robot_ctrl, std::shared_ptr<RobotContext> ctx,
  double loop_rate_s) :
  Node(fsm_name),
  robot_ctrl_(robot_ctrl), ctx_(ctx), blocked_(true) {

  // Convert seconds to milliseconds for ROS 2 wall timer
  auto millis = std::chrono::milliseconds(static_cast<int>(loop_rate_s * 1000.0));

  timer_ = this->create_wall_timer(
    millis, 
    std::bind(&FiniteStateMachine::control_callback, this)
  );

  RCLCPP_INFO(this->get_logger(), "FSM: [%s] initialized at %.2f Hz.", fsm_name.c_str(), 1.0/loop_rate_s);
}

/**
 * @brief Registers a new state and its allowed transitions in the FSM.
 * Automatically injects node dependencies into the state instance.
 */
void FiniteStateMachine::add_state(std::shared_ptr<State> state, TransitionTable transitions) {
  if (!state) {
    RCLCPP_ERROR(this->get_logger(), "Attempted to add a null state!");
    return;
  }

  std::string alias = state->get_alias();

  // Prevent duplicate registration
  if (states_registry_.find(alias) != states_registry_.end()) {
    RCLCPP_WARN(this->get_logger(), "State '%s' already exists. Registration skipped.", alias.c_str());
    return;
  }

  // Dependency injection: providing the state with the tools it needs to operate
  state->setup(robot_ctrl_, this->get_logger(), this->get_clock(), ctx_);

  states_registry_[alias] = state;
  state_transitions_[alias] = transitions;

  // Set the first registered state as the default entry point
  if (current_state_alias_.empty()) {
    current_state_alias_ = alias;
    RCLCPP_INFO(this->get_logger(), "FSM Entry Point defined as: [%s]", alias.c_str());
  }
}

/**
 * @brief Unblocks the execution loop to begin state processing.
 */
void FiniteStateMachine::start() {
  if (current_state_alias_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "FSM Start Failed: No states registered in the registry.");
    return;
  }
  blocked_ = false;
  RCLCPP_INFO(this->get_logger(), "FSM Execution Loop: STARTED.");
}

/**
 * @brief Processes external or internal events to trigger state transitions.
 */
void FiniteStateMachine::handle_event(const std::string& event) {
  auto state_it = state_transitions_.find(current_state_alias_);

  if (state_it != state_transitions_.end()) {
    auto& transitions = state_it->second;
    auto event_it = transitions.find(event);

    if (event_it != transitions.end()) {
      std::string next_state_alias = event_it->second;

      RCLCPP_INFO(this->get_logger(), "TRANSITION: [%s] --(%s)--> [%s]",
        current_state_alias_.c_str(), event.c_str(), next_state_alias.c_str());

      // Trigger cleanup logic of the current state before leaving
      states_registry_[current_state_alias_]->reset_lifecycle();

      // Switch to the new state
      current_state_alias_ = next_state_alias;
      last_event_ = event;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Event '%s' ignored in state '%s' (No transition defined).", 
        event.c_str(), current_state_alias_.c_str());
    }
  }
}

/**
 * @brief Periodic timer callback that drives the active state logic.
 */
void FiniteStateMachine::control_callback() {
  // Safety check: skip execution if FSM is blocked or not yet started
  if (blocked_) return;

  auto it = states_registry_.find(current_state_alias_);
  if (it != states_registry_.end()) {
    /**
     * The execute() call handles both the entry logic (first run) 
     * and the periodic update logic of the concrete state.
     */
    auto output_event = it->second->execute();

    // If the state logic returns an event string, process the transition
    if (output_event.has_value()) { 
      handle_event(output_event.value()); 
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "FSM Runtime Error: Current state '%s' not found in registry!", 
      current_state_alias_.c_str());
  }
}