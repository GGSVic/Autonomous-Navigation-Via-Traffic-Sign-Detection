#ifndef ALL_STATES_HPP
#define ALL_STATES_HPP

#include "puzzlebot_controller/utils/chronometer.hpp"
#include "state.hpp"
#include <random>
#include <unordered_map>

// --- 1. FLOW STATES (General motion) ---

class FreeFlow : public State {
public:
  FreeFlow() : State("FREE_FLOW") {}

protected:
  void on_entry() override {
    robot_ctrl_->set_linear_speed(0.5);
    robot_ctrl_->resume_line_following();
  }
};

class CrossApproach : public State {
public:
  CrossApproach() : State("CROSS_APPROACH") {}

protected:
  void on_entry() override { robot_ctrl_->set_linear_speed(0.3); }
};

class CautionFlow : public State {
public:
  CautionFlow() : State("CAUTION_FLOW") {}

protected:
  void on_entry() override {
    robot_ctrl_->set_linear_speed(0.3);
    if (!timer_)
      timer_ = std::make_shared<Chronometer>(clock_);
    timer_->start(3.5);
  }
  std::optional<std::string> on_execute() override {
    if (timer_ && timer_->timeout_reached())
      return "timeout";
    return std::nullopt;
  }

private:
  std::shared_ptr<Chronometer> timer_;
};

// --- 2. INTERSECTION HUB & MANEUVERS ---

class ManeuverHub : public State {
public:
  ManeuverHub() : State("MANEUVER_HUB") {}

protected:
  std::optional<std::string> on_execute() override {
    if (ctx_->direction.has_value())
      return ctx_->direction->sign_name;
    return std::nullopt;
  }
};

class MStraight : public State {
public:
  MStraight() : State("M_STRAIGHT") {}

protected:
  void on_entry() override {
    robot_ctrl_->pause_line_following();
    robot_ctrl_->maneuver_straight();
  }
  std::optional<std::string> on_execute() override {
    if (ctx_->is_path_recovered)
      return "done";
    return std::nullopt;
  }

  void on_exit() override { robot_ctrl_->resume_line_following(); }
};

class MTurnRight : public State {
public:
  MTurnRight() : State("M_TURN_RIGHT") {}

protected:
  void on_entry() override {
    robot_ctrl_->pause_line_following();
    robot_ctrl_->turn_right();
  }
  std::optional<std::string> on_execute() override {
    if (ctx_->is_path_recovered)
      return "done";
    return std::nullopt;
  }
  void on_exit() override { robot_ctrl_->resume_line_following(); }
};

class MTurnLeft : public State {
public:
  MTurnLeft() : State("M_TURN_LEFT") {}

protected:
  void on_entry() override {
    robot_ctrl_->pause_line_following();
    robot_ctrl_->turn_left();
  }
  std::optional<std::string> on_execute() override {
    if (ctx_->is_path_recovered)
      return "done";
    return std::nullopt;
  }
  void on_exit() override { robot_ctrl_->resume_line_following(); }
};

class MUTurn : public State {
public:
  MUTurn() : State("M_U_TURN") {}

protected:
  void on_entry() override {
    robot_ctrl_->pause_line_following();
    robot_ctrl_->rotate_right();
  }
  std::optional<std::string> on_execute() override {
    if (ctx_->is_path_recovered)
      return "done";
    return std::nullopt;
  }
  void on_exit() override { robot_ctrl_->resume_line_following(); }
};

// --- 3. TRAFFIC CONTROL STATES ---

class TrafficHub : public State {
public:
  TrafficHub() : State("TRAFFIC_HUB") {}

protected:
  std::optional<std::string> on_execute() override {
    auto tf = ctx_->tf_light;
    auto reg = ctx_->regulatory;
    if (!tf.has_value() && !reg.has_value())
      return std::nullopt;

    puzzlebot_interfaces::msg::TrafficSignDetection sign;
    if (tf.has_value() && reg.has_value()) {
      sign = (tf->timestamp.sec > reg->timestamp.sec) ? *tf : *reg;
    } else {
      sign = tf.has_value() ? *tf : *reg;
    }

    static const std::unordered_map<std::string, std::string> actions = {
        {"tf_red", "wait_for_green"},
        {"stop", "stop"},
        {"give_way", "yield"},
        {"tf_green", "proceed"},
        {"tf_yellow", "proceed"}};

    if (actions.count(sign.sign_name))
      return actions.at(sign.sign_name);
    return "proceed";
  }
};

class WaitLight : public State {
public:
  WaitLight() : State("WAIT_LIGHT") {}

protected:
  void on_entry() override {
    robot_ctrl_->pause_line_following();
    robot_ctrl_->stop();
  }
  std::optional<std::string> on_execute() override {
    if (ctx_->tf_light.has_value() && ctx_->tf_light->sign_name == "tf_green")
      return "proceed";
    return std::nullopt;
  }
  void on_exit() override { robot_ctrl_->resume_line_following(); }
};

class WaitYield : public State {
public:
  WaitYield() : State("WAIT_YIELD") {}

protected:
  void on_entry() override {
    if (!timer_)
      timer_ = std::make_shared<Chronometer>(clock_);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::bernoulli_distribution d(0.5);

    if (d(gen)) {
      RCLCPP_INFO(logger_, "WaitYield: Anticipating obstacle. Stopping.");
      robot_ctrl_->pause_line_following();
      robot_ctrl_->stop();
      std::uniform_real_distribution<double> wait_dist(1.5, 4.5);
      timer_->start(wait_dist(gen));
    } else {
      RCLCPP_INFO(logger_, "WaitYield: Path appears clear.");
      timer_->start(0.0);
    }
  }
  std::optional<std::string> on_execute() override {
    if (timer_ && timer_->timeout_reached())
      return "proceed";
    return std::nullopt;
  }
  void on_exit() override { robot_ctrl_->resume_line_following(); }

private:
  std::shared_ptr<Chronometer> timer_;
};

class WaitStop : public State {
public:
  WaitStop() : State("WAIT_STOP") {}

protected:
  void on_entry() override {
    if (!timer_)
      timer_ = std::make_shared<Chronometer>(clock_);
    robot_ctrl_->pause_line_following();
    robot_ctrl_->stop();
    timer_->start(3.0);
  }
  std::optional<std::string> on_execute() override {
    if (timer_ && timer_->timeout_reached())
      return "proceed";
    return std::nullopt;
  }
  void on_exit() override { robot_ctrl_->resume_line_following(); }

private:
  std::shared_ptr<Chronometer> timer_;
};

#endif // ALL_STATES_HPP