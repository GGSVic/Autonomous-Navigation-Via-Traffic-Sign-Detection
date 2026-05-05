#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "puzzlebot_controller/controller/puzzlebot_controller.hpp"
#include "puzzlebot_controller/navigator_system.hpp"
#include "puzzlebot_controller/state_machine/all_states.hpp"
#include "puzzlebot_controller/utils/robot_context.hpp"

/**
 * @file main.cpp
 * @brief Entry point for the Puzzlebot Navigation Stack.
 *
 * This application orchestrates the three main pillars of the robot:
 * 1. Hardware Control (PuzzlebotController)
 * 2. Decision Making (FiniteStateMachine)
 * 3. Perception Integration (NavigatorSystem)
 */

int main(int argc, char **argv) {
  // Initialize ROS 2 communication infrastructure
  rclcpp::init(argc, argv);

  // Resource Allocation
  // RobotContext acts as a thread-safe blackboard for cross-node data sharing
  auto ctx = std::make_shared<RobotContext>();

  // Controller handles /cmd_vel and PID steering
  auto ctrl = std::make_shared<PuzzlebotController>();

  // Behavioral Logic Setup
  // We run the FSM at 10Hz (0.1s) for responsive behavior switching
  auto fsm = std::make_shared<FiniteStateMachine>("fsm_node", ctrl, ctx, 0.1);

  // State Registry & Transition Map
  // Logic: Current State -> { { "Event", "Next State" }, ... }

  // -- Global Navigation & Perception States --
  fsm->add_state(std::make_shared<FreeFlow>(),
                 {{"tf_red", "CROSS_APPROACH"},
                  {"tf_yellow", "CROSS_APPROACH"},
                  {"give_way", "CROSS_APPROACH"},
                  {"stop", "CROSS_APPROACH"},
                  {"max_speed", "CAUTION_FLOW"},
                  {"roadwork", "CAUTION_FLOW"},
                  {"street_crossing", "MANEUVER_HUB"}});

  fsm->add_state(
      std::make_shared<CrossApproach>(),
      {{"street_crossing", "TRAFFIC_HUB"}, {"tf_green", "FREE_FLOW"}});

  fsm->add_state(std::make_shared<CautionFlow>(),
                 {{"timeout", "FREE_FLOW"},
                  {"tf_red", "CROSS_APPROACH"},
                  {"tf_yellow", "CROSS_APPROACH"},
                  {"give_way", "CROSS_APPROACH"},
                  {"stop", "CROSS_APPROACH"},
                  {"street_crossing", "MANEUVER_HUB"}});

  // -- Intersection Handling (Maneuver Hub) --
  fsm->add_state(std::make_shared<ManeuverHub>(),
                 {{"go_ahead", "M_STRAIGHT"},
                  {"turn_right", "M_TURN_RIGHT"},
                  {"turn_left", "M_TURN_LEFT"},
                  {"roundabout", "M_U_TURN"}});

  // -- Specific Kinetic Maneuvers --
  fsm->add_state(std::make_shared<MStraight>(), {{"done", "FREE_FLOW"}});
  fsm->add_state(std::make_shared<MTurnRight>(), {{"done", "FREE_FLOW"}});
  fsm->add_state(std::make_shared<MTurnLeft>(), {{"done", "FREE_FLOW"}});
  fsm->add_state(std::make_shared<MUTurn>(), {{"done", "FREE_FLOW"}});

  // -- Traffic Logic Hub (Wait/Yield logic) --
  fsm->add_state(std::make_shared<TrafficHub>(),
                 {{"yield", "WAIT_YIELD"},
                  {"stop", "WAIT_STOP"},
                  {"wait_for_green", "WAIT_LIGHT"},
                  {"proceed", "MANEUVER_HUB"}});

  // -- Temporal/Conditional Wait States --
  fsm->add_state(std::make_shared<WaitYield>(), {{"proceed", "MANEUVER_HUB"}});
  fsm->add_state(std::make_shared<WaitStop>(), {{"proceed", "MANEUVER_HUB"}});
  fsm->add_state(std::make_shared<WaitLight>(), {{"proceed", "MANEUVER_HUB"}});

  // Activate Behavioral Engine
  fsm->start();

  // Perception Integration
  // The Navigator binds the FSM, Context, and Controller to ROS 2 topics
  auto nav = std::make_shared<NavigatorSystem>(ctrl, fsm, ctx);

  // Multi-threaded Execution Engine
  /**
   * We assign 3 threads to the executor:
   * Thread 1: Dedicated to Controller updates (PID & /cmd_vel).
   * Thread 2: Dedicated to FSM logic transitions (10Hz cycles).
   * Thread 3: Dedicated to high-frequency Vision Subscriptions (Navigator).
   */
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    3);

  executor.add_node(ctrl);
  executor.add_node(fsm);
  executor.add_node(nav);

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Puzzlebot Navigator System is now Spinning...");

  try {
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "System Crash: %s", e.what());
  }

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Shutting down Puzzlebot Navigation...");
  rclcpp::shutdown();
  return 0;
}