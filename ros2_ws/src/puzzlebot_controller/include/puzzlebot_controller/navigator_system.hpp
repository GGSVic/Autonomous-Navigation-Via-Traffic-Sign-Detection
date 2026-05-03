#ifndef NAVIGATOR_SYSTEM_HPP
#define NAVIGATOR_SYSTEM_HPP

#include "puzzlebot_controller/controller/puzzlebot_controller.hpp"
#include "puzzlebot_controller/state_machine/state_machine.hpp"
#include "puzzlebot_controller/utils/robot_context.hpp"
#include "puzzlebot_interfaces/msg/road_perception.hpp"
#include "puzzlebot_interfaces/msg/traffic_sign_detection.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class NavigatorSystem
 * @brief Top-level integration node bridging perception with decision-making and control.
 *
 * This node acts as the central hub of the Puzzlebot stack. It performs three critical roles:
 * 1. Data Ingestion: Subscribes to processed vision streams.
 * 2. Context Update: populates the RobotContext (Blackboard) with environmental data.
 * 3. Event Dispatching: Notifies the Finite State Machine (FSM) of significant cues 
 *    (e.g., stop signs, crosswalks/street_crossing) to trigger transitions.
 */
class NavigatorSystem : public rclcpp::Node {
public:
  /**
   * @brief Initializes the navigator with its core dependencies.
   * 
   * @param[in] controller Actuator interface for motion execution.
   * @param[in] fsm Decision-making engine for behavioral transitions.
   * @param[in] ctx Shared memory container for perception data.
   */
  NavigatorSystem(std::shared_ptr<PuzzlebotController> controller,
    std::shared_ptr<FiniteStateMachine> fsm, std::shared_ptr<RobotContext> ctx);

private:
  /**
   * @brief Configures ROS 2 subscribers with appropriate QoS profiles.
   *
   * Utilizes SensorData QoS for low-latency delivery of vision messages.
   */
  void setup_communication();

  /**
   * @brief Callback for processing traffic sign detections.
   * 
   * Categorizes detected signs into the context and dispatches transition events 
   * to the FSM based on the sign's semantic meaning.
   * 
   * @param[in] msg Shared pointer to the incoming TrafficSignDetection message.
   */
  void traffic_sign_callback(
    const puzzlebot_interfaces::msg::TrafficSignDetection::SharedPtr msg);

  /**
   * @brief Callback for processing line and road feature detections.
   * 
   * Updates the controller's steering setpoint and handles landmark-based 
   * events, such as reaching a crosswalk.
   * 
   * @param[in] msg Shared pointer to the incoming RoadPerception message.
   */
  void line_callback(const puzzlebot_interfaces::msg::RoadPerception::SharedPtr msg);

  // --- CORE SYSTEM POINTERS ---
  /// Pointer to the behavioral orchestrator.
  std::shared_ptr<FiniteStateMachine> fsm_;
  
  /// Pointer to the shared data container (Blackboard).
  std::shared_ptr<RobotContext> ctx_;
  
  /// Pointer to the motion controller interface.
  std::shared_ptr<PuzzlebotController> ctrl_;

  // --- SUBSCRIBERS ---
  /// Subscription for semantic traffic sign information.
  rclcpp::Subscription<puzzlebot_interfaces::msg::TrafficSignDetection>::SharedPtr sign_sub_;
  
  /// Subscription for line geometry and road infrastructure data.
  rclcpp::Subscription<puzzlebot_interfaces::msg::RoadPerception>::SharedPtr line_sub_;
};

#endif // NAVIGATOR_SYSTEM_HPP