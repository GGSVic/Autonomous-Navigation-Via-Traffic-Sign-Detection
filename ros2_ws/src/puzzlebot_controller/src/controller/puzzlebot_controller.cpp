#include "puzzlebot_controller/controller/puzzlebot_controller.hpp"

using namespace std;
using namespace geometry_msgs::msg;

/**
 * @brief Constructor for the Puzzlebot Controller.
 * Initializes the steering logic with tuned gains for typical warehouse
 * environments.
 */
PuzzlebotController::PuzzlebotController(std::string node_name,
                                         double min_linear_speed)
    : Node(node_name), min_linear_speed(min_linear_speed),
      target_linear_speed_(0.0), is_tracking_line(true) {

  RCLCPP_INFO(this->get_logger(), "Initializing Puzzlebot Controller Node...");

  // Standard ROS 2 setup: High frequency / cmd_vel for motor drivers
  cmd_vel_pub = this->create_publisher<Twist>("/cmd_vel", 10);

  /**
   * PID and Coupling Gain Tuning:
   * Kp = 1.2, Ki = 0.0, Kd = 0.15
   * Coupling Gain = 1.2 (Moderate speed reduction during aggressive turns)
   */
  steering_ctrl = SteeringController(1.2, 0.0, 0.15, 1.2,
                                     this->get_clock()->now().seconds());

  // Set operational limits: [Min linear, Max linear, Max angular]
  steering_ctrl.set_speed_limits(min_linear_speed, 1.0, 0.8);

  RCLCPP_INFO(this->get_logger(),
              "Puzzlebot controller successfully initialized.");
}

/**
 * @brief Resumes the line following behavior and clears PID accumulators.
 */
void PuzzlebotController::resume_line_following() {
  is_tracking_line = true;
  RCLCPP_INFO(this->get_logger(), "Control Mode: [LINE_TRACKING]");

  // Clear integral/derivative error to prevent 'jumps' from stale time deltas
  steering_ctrl.reset(this->get_clock()->now().seconds());
}

/**
 * @brief Pauses line following to allow manual or open-loop maneuvers.
 */
void PuzzlebotController::pause_line_following() {
  is_tracking_line = false;
  RCLCPP_INFO(this->get_logger(), "Control Mode: [MANUAL_MANEUVERS]");
}

/**
 * @brief Computes and publishes velocity commands based on perception input.
 */
void PuzzlebotController::follow_line_path(double current_steering_angle) {
  if (!is_tracking_line)
    return;

  double linear_output, angular_output;
  steering_ctrl.compute_steering_response(
      current_steering_angle, this->get_clock()->now().seconds(),
      target_linear_speed_, linear_output, angular_output);

  publish_velocity(linear_output, angular_output);
}

/**
 * @brief Safe straight motion. Requires line tracking to be paused.
 */
void PuzzlebotController::maneuver_straight() {
  if (is_tracking_line) {
    RCLCPP_WARN(get_logger(),
                "Direct maneuver ignored: Line Tracking is active.");
    return;
  }
  publish_velocity(0.45, 0.0);
}

/**
 * @brief Arc turn to the right. Requires line tracking to be paused.
 */
void PuzzlebotController::turn_right() {
  if (is_tracking_line) {
    RCLCPP_WARN(get_logger(),
                "Direct maneuver ignored: Line Tracking is active.");
    return;
  }
  publish_velocity(0.3, -0.7);
}

/**
 * @brief Arc turn to the left. Requires line tracking to be paused.
 */
void PuzzlebotController::turn_left() {
  if (is_tracking_line) {
    RCLCPP_WARN(get_logger(),
                "Direct maneuver ignored: Line Tracking is active.");
    return;
  }
  publish_velocity(0.3, 0.7);
}

/**
 * @brief Spin clockwise on the robot's center axis.
 */
void PuzzlebotController::rotate_right() {
  if (is_tracking_line) {
    RCLCPP_WARN(get_logger(),
                "Direct rotation ignored: Line Tracking is active.");
    return;
  }
  publish_velocity(0.0, -0.7);
}

/**
 * @brief Spin counter-clockwise on the robot's center axis.
 */
void PuzzlebotController::rotate_left() {
  if (is_tracking_line) {
    RCLCPP_WARN(get_logger(),
                "Direct rotation ignored: Line Tracking is active.");
    return;
  }
  publish_velocity(0.0, 0.7);
}

/**
 * @brief Emergency/Planned stop. Forcefully halts motors.
 * @note This method bypasses the is_tracking_line check for safety.
 */
void PuzzlebotController::stop() {
  // We force a stop regardless of state for safety-critical situations
  publish_velocity(0.0, 0.0);
}

/**
 * @brief Updates the target forward speed (setpoint).
 */
void PuzzlebotController::set_linear_speed(double speed) {
  target_linear_speed_ = speed;
}

// --- Internal Publishing Helpers ---

/**
 * @brief Composes and broadcasts the Twist message to /cmd_vel.
 * @param linear Linear velocity (x-axis) in m/s.
 * @param angular Angular velocity (z-axis) in rad/s.
 */
void PuzzlebotController::publish_velocity(double linear, double angular) {
  twist_cmd.linear.x = linear;
  twist_cmd.angular.z = angular;
  cmd_vel_pub->publish(twist_cmd);
}

void PuzzlebotController::publish_linear_velocity(double linear) {
  twist_cmd.linear.x = linear;
  cmd_vel_pub->publish(twist_cmd);
}

void PuzzlebotController::publish_angular_velocity(double angular) {
  twist_cmd.angular.z = angular;
  cmd_vel_pub->publish(twist_cmd);
}