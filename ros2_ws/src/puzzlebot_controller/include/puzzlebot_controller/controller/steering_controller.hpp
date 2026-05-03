#pragma once

#ifndef STEERING_CONTROLLER_HPP
#define STEERING_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <iostream>

/**
 * @class SteeringController
 * @brief Professional PID implementation for path following and velocity coupling.
 * 
 * Optimized for Puzzlebot kinematics. This controller translates lateral angular errors
 * into steering commands and dynamically adjusts linear speed to maintain stability 
 * during sharp maneuvers.
 */
class SteeringController {

public:
  /**
   * @brief Default constructor. 
   * Initializes all gains and accumulators to zero to ensure a deterministic initial state.
   */
  SteeringController() noexcept :
    kp(0), ki(0), kd(0), coupling_gain(0), max_angular(0), min_linear(0), max_linear(0),
    integral(0), derivative(0), prev_error(0), prev_time(0) {}

  /**
   * @brief Parameterized constructor with control gains.
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @param cg Coupling gain (Speed reduction factor during turns).
   * @param start_time Initial system timestamp in seconds.
   */
  SteeringController(double p, double i, double d, double cg, double start_time) noexcept :
    kp(p), ki(i), kd(d), coupling_gain(cg), max_angular(1.0), min_linear(0.0), max_linear(1.0),
    integral(0.0), derivative(0.0), prev_error(0.0), prev_time(start_time) {}

  /**
   * @brief Configures physical operational constraints.
   * @param min_lin Minimum linear velocity (m/s).
   * @param max_lin Maximum linear velocity (m/s).
   * @param max_ang Maximum angular velocity (rad/s).
   */
  void set_speed_limits(double min_lin, double max_lin, double max_ang) noexcept {
    min_linear = min_lin;
    max_linear = max_lin;
    max_angular = max_ang;
  }

  /**
   * @brief High-level interface to update the controller and retrieve motion commands.
   * 
   * Processes the current perception data to generate a synchronized Twist-like response.
   * 
   * @param[in] current_angle Detected line angle in radians.
   * @param[in] current_time Current system time (s).
   * @param[in] base_linear Target forward speed (m/s).
   * @param[out] out_linear Resulting coupled linear velocity (m/s).
   * @param[out] out_angular Resulting PID-corrected angular velocity (rad/s).
   */
  void compute_steering_response(double current_angle, double current_time, double base_linear,
    double& out_linear, double& out_angular) {
    out_angular = compute_steering_correction(current_angle, current_time);
    out_linear = compute_coupled_linear_velocity(base_linear, out_angular);
  }

  /**
   * @brief Resets PID accumulators and updates the internal clock.
   * Use this when re-acquiring a path or changing states to prevent integral windup.
   * @param current_time Current system time (s).
   */
  void reset(double current_time) noexcept {
    integral = 0.0;
    derivative = 0.0;
    prev_error = 0.0;
    prev_time = current_time;
  }

private:
  double kp, ki, kd, coupling_gain;
  double max_angular, min_linear, max_linear;
  double integral, derivative, prev_error, prev_time;

  /**
   * @brief Internal PID mathematics.
   * 
   * Uses a cosine-based error function: error = cos(theta). 
   * This provides a smooth, non-linear error mapping where 0 radians (centered)
   * produces a null error, and deviations produce corrective torques.
   * 
   * @return Clamped angular velocity command (rad/s).
   */
  double compute_steering_correction(double current_angle, double current_time) {
    double dt = current_time - prev_time;
    if (dt <= 0.0) return 0.0;

    // Line is centered when angle is at PI/2 (cos(PI/2) = 0)
    double error = std::cos(current_angle);
    integral += error * dt;
    derivative = (error - prev_error) / dt;

    // Negative gain applied to ensure the robot turns TOWARDS the error
    double raw_output = -1.0 * (kp * error) + (ki * integral) + (kd * derivative);

    prev_error = error;
    prev_time = current_time;

    return std::clamp(raw_output, -max_angular, max_angular);
  }

  /**
   * @brief Dynamic velocity scaler (Coupling).
   * 
   * Reduces linear velocity as a function of angular effort to prevent
   * centrifugal instability during sharp turns.
   * 
   * @param desired_linear Base forward speed (m/s).
   * @param steering_output Current angular effort (rad/s).
   * @return Clamped linear velocity (m/s).
   */
  double compute_coupled_linear_velocity(double desired_linear, double steering_output) {
    // Parabolic reduction: v = v_base - gain * (omega^2)
    double steering_term = coupling_gain * std::pow(std::abs(steering_output), 2);
    double computed_v = desired_linear - steering_term;

    return std::clamp(computed_v, min_linear, max_linear);
  }
};

#endif