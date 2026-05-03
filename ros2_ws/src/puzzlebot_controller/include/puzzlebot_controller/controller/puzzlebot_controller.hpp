#ifndef PUZZLEBOT_CONTROLLER_HPP_
#define PUZZLEBOT_CONTROLLER_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "puzzlebot_controller/controller/steering_controller.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class PuzzlebotController
 * @brief High-level motion and navigation orchestrator for the Puzzlebot.
 * 
 * This node acts as the bridge between perception (vision) and actuators (motors).
 * It manages the robot's movement, allowing switching between closed-loop 
 * line following and open-loop maneuvers.
 */
class PuzzlebotController : public rclcpp::Node {
public:
  /**
   * @param node_name ROS 2 node name.
   * @param min_linear_speed Minimum speed (m/s) required to overcome motor static friction.
   */
  explicit PuzzlebotController(
    std::string node_name = "puzzlebot_controller", 
    double min_linear_speed = 0.3);

  /** @brief Enables the closed-loop line tracking logic. */
  void resume_line_following();

  /** @brief Disables line tracking and resets PID/internal controller states. */
  void pause_line_following();

  /**
   * @brief Executes closed-loop motion based on vision-detected angle.
   * @note Only effective if is_tracking_line is true.
   * @param[in] current_steering_angle Target line angle in radians relative to the robot's heading.
   */
  void follow_line_path(double current_steering_angle);

  /** @brief Performs open-loop straight motion at target_linear_speed_ (m/s). */
  void maneuver_straight();

  /** @brief Performs a smooth right turn with constant curvature. */
  void turn_right();

  /** @brief Performs a smooth left turn with constant curvature. */
  void turn_left();

  /** @brief Performs an in-place rotation (0 linear velocity) to the right. */
  void rotate_right();

  /** @brief Performs an in-place rotation (0 linear velocity) to the left. */
  void rotate_left();

  /** @brief Emergency or planned stop. Sets all velocities to 0.0. */
  void stop();

  /** 
   * @brief Updates the nominal forward speed setpoint. 
   * @param[in] speed New target linear velocity in m/s.
   */
  void set_linear_speed(double speed);

private:
  /// Current velocity command state.
  geometry_msgs::msg::Twist twist_cmd;
  
  /// Velocity command publisher to the hardware interface (usually /cmd_vel).
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  /// Core logic for calculating angular responses based on lateral errors.
  SteeringController steering_ctrl;

  double min_linear_speed;      ///< Minimum m/s to prevent motor stalling.
  double target_linear_speed_;  ///< Desired forward velocity in m/s.
  bool is_tracking_line;        ///< Control state flag.

  /** 
   * @brief Composes and sends a Twist message.
   * @param[in] linear Desired linear x velocity (m/s).
   * @param[in] angular Desired angular z velocity (rad/s).
   */
  void publish_velocity(double linear, double angular);

  /** @brief Updates the linear component of the current command and publishes. */
  void publish_linear_velocity(double linear);

  /** @brief Updates the angular component of the current command and publishes. */
  void publish_angular_velocity(double angular);
};

#endif // PUZZLEBOT_CONTROLLER_HPP_