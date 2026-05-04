#include "puzzlebot_controller/navigator_system.hpp"

/**
 * @brief Constructor for the NavigatorSystem.
 * Integrates the controller, FSM, and shared context into a single ROS 2
 * gateway.
 */
NavigatorSystem::NavigatorSystem(
    std::shared_ptr<PuzzlebotController> controller,
    std::shared_ptr<FiniteStateMachine> fsm, std::shared_ptr<RobotContext> ctx)
    : Node("navigator_system"), fsm_(fsm), ctx_(ctx), ctrl_(controller) {

  // Setup ROS 2 communication infrastructure
  setup_communication();

  RCLCPP_INFO(this->get_logger(),
              "Navigator System: [ACTIVE]. Waiting for perception streams.");
}

/**
 * @brief Configures subscribers and implements a blocking wait for publishers.
 *
 * This ensures the robot doesn't start its logic until the vision pipeline
 * (traffic signs and road perception) is fully operational.
 */
void NavigatorSystem::setup_communication() {
  auto sensor_qos = rclcpp::SensorDataQoS();
  std::string sign_topic = "/traffic_sign_tracker/results";
  std::string line_topic = "/path_interpreter/results";

  auto wait_for_topic = [this](const std::string &topic_name) {
    RCLCPP_INFO(this->get_logger(), "Connecting to: %s ...",
                topic_name.c_str());
    while (this->count_publishers(topic_name) == 0) {
      if (!rclcpp::ok())
        return;
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_INFO(this->get_logger(), "Connection established with: %s",
                topic_name.c_str());
  };

  // Block initialization until perception nodes are ready
  wait_for_topic(sign_topic);
  wait_for_topic(line_topic);

  sign_sub_ = this->create_subscription<
      puzzlebot_interfaces::msg::TrafficSignDetection>(
      sign_topic, sensor_qos,
      std::bind(&NavigatorSystem::traffic_sign_callback, this,
                std::placeholders::_1));

  line_sub_ =
      this->create_subscription<puzzlebot_interfaces::msg::RoadPerception>(
          line_topic, sensor_qos,
          std::bind(&NavigatorSystem::line_callback, this,
                    std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "All perception subscribers are linked.");
}

/**
 * @brief Callback for Traffic Sign detections.
 *
 * Acts as an event dispatcher:
 * 1. Categorizes the sign into the shared RobotContext (Blackboard).
 * 2. Notifies the FSM to trigger transitions based on the sign's identity.
 */
void NavigatorSystem::traffic_sign_callback(
    const puzzlebot_interfaces::msg::TrafficSignDetection::SharedPtr msg) {

  // Thread-safe access to the blackboard
  std::lock_guard<std::mutex> lock(ctx_->memory_mutex);

  // Trigger behavior change based on sign name (e.g., "stop", "turn_left")
  fsm_->handle_event(msg->sign_name);

  // Update short-term memory categories
  switch (msg->category) {
  case puzzlebot_interfaces::msg::TrafficSignDetection::DIRECTION:
    ctx_->direction = *msg;
    break;
  case puzzlebot_interfaces::msg::TrafficSignDetection::TRAFFIC_LIGHT:
    ctx_->tf_light = *msg;
    break;
  case puzzlebot_interfaces::msg::TrafficSignDetection::REGULATORY:
    ctx_->regulatory = *msg;
    break;
  case puzzlebot_interfaces::msg::TrafficSignDetection::CAUTION:
    ctx_->caution = *msg;
    break;
  default:
    RCLCPP_DEBUG(this->get_logger(), "Unknown sign category received.");
    break;
  }
}

/**
 * @brief Callback for Road Perception (Lines and Crosswalks).
 *
 * Handles high-frequency steering and specialized landmark events like
 * intersections.
 */
void NavigatorSystem::line_callback(
    const puzzlebot_interfaces::msg::RoadPerception::SharedPtr msg) {

  if (msg->crosswalk_detected) {
    std::lock_guard<std::mutex> lock(ctx_->memory_mutex);
    ctx_->is_path_recovered = false;
    // Synchronize perception time with detection history
    rclcpp::Time current_time = msg->timestamp;

    if (ctx_->direction.has_value()) {
      double data_age = (current_time - ctx_->direction->timestamp).seconds();

      // Temporal Filtering: Ignore signs from previous intersections (> 3.0s
      // ago)
      if (data_age > 3.0) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Stale direction data (%.2f s). Forcing 'go_ahead'.",
                     data_age);
        ctx_->direction->sign_name = "go_ahead";
      }
    } else {
      // Default behavior if no sign was seen: keep going straight
      puzzlebot_interfaces::msg::TrafficSignDetection default_dir;
      default_dir.sign_name = "go_ahead";
      ctx_->direction = default_dir;
    }

    // Trigger FSM event for intersection handling
    fsm_->handle_event("street_crossing");

  } else if (msg->target_detected) {
    if (ctx_->is_path_recovered == false) {
      // Validation: Use your existing logic (verticality and steering)
      // to decide if this frame is a "good" candidate for recovery.
      if (msg->target.verticality > 0.4 && msg->target.steering_angle > 1.0) {
        ctx_->recovery_hits++;
      } else {
        // Reset: If the frame is bad (e.g., a zebra crossing), we start
        // over.
        ctx_->recovery_hits = 0;
        return;
      }

      // Threshold check: Only recover after 'n' consistent good frames.
      if (ctx_->recovery_hits < ctx_->HIT_THRESHOLD) {
        return;
      }

      // Success: Path is stable.
      ctx_->is_path_recovered = true;
      ctx_->recovery_hits = 0;
    }

    // Normal Operation: Feed steering angle to the PID controller
    ctrl_->follow_line_path(msg->target.steering_angle);
  }
}