#pragma once
#include <cmath>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "tinyfsm.hpp"

#pragma region Finite State Machine
// Finite State Machine Events
struct WakeUpEvent : tinyfsm::Event {};
struct CommandVelocityEvent : tinyfsm::Event {
  double linear_x;
  double angular_z;
  CommandVelocityEvent(double lx = 0.0, double az = 0.0)
      : linear_x(lx), angular_z(az) {}
};

/* ----------- bridge (FSM -> ROS side effects) ----------- */
/* node will set these callbacks so FSM code can call them in transition actions
 */
struct HexapodBridge {
  static std::function<void()> sendStandPose;
  static std::function<void(bool)> startWalkCycle;
  static std::function<void()> cancelLegActions;
  static std::function<bool()> isWalkingReversed;
};

class StateMachine : public tinyfsm::Fsm<StateMachine> {
 public:
  void react(tinyfsm::Event const& e) {}  // default empty reaction
  virtual void react(WakeUpEvent const& e) {}
  virtual void react(CommandVelocityEvent const& e) {}
  virtual void entry() {}
  virtual void exit() {}
};
#pragma endregion Finite State Machine

// States
#pragma region States
class Rest;
class Standing;
class Walking;
constexpr double VELOCITY_DEADZONE = 0.01;

class Rest : public StateMachine {
 public:
  using StateMachine::react;

  void entry() override {
    RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Entering Rest state");
  }

  void react(WakeUpEvent const& e) override {
    RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Transitioning to Standing state");
    auto action = []() { HexapodBridge::sendStandPose(); };
    transit<Standing>(action);
  }
};

class Standing : public StateMachine {
 public:
  using StateMachine::react;

  void entry() override {
    RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Entering Standing state");
  }

  void react(CommandVelocityEvent const& e) override {
    if (std::abs(e.linear_x) > VELOCITY_DEADZONE) {
      RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Transitioning to Walking state");
      bool is_reversed = e.linear_x < 0;
      auto action = [is_reversed]() {
        HexapodBridge::startWalkCycle(is_reversed);
      };
      transit<Walking>(action);
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger("hexapod_controller"), "Ignoring insignificant velocity changes");
    }
  }
};

class Walking : public StateMachine {
 public:
  using StateMachine::react;

  void entry() override {
    RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Entering Walking state");
  }

  void react(CommandVelocityEvent const& e) override {
    const bool forward_cmd = std::abs(e.linear_x) >= VELOCITY_DEADZONE;
    const bool turn_cmd = std::abs(e.angular_z) >= VELOCITY_DEADZONE;

    if (!forward_cmd && !turn_cmd) {
      RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Transitioning to Standing state");
      auto action = []() {
        HexapodBridge::cancelLegActions();
        HexapodBridge::sendStandPose();
      };
      transit<Standing>(action);
      return;
    }

    if (!forward_cmd) {
      // the gait engine has no angular velocity support yet: a turn command
      // must neither stop the robot nor restart the straight gait
      static rclcpp::Clock warn_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("hexapod_controller"), warn_clock, 5000,
          "Turning is not supported yet, ignoring angular command");
      return;
    }

    const bool want_reversed = e.linear_x < 0;
    if (HexapodBridge::isWalkingReversed() == want_reversed) {
      RCLCPP_DEBUG(rclcpp::get_logger("hexapod_controller"),
                   "Walk direction unchanged, continuing the current gait cycle");
      return;
    }

    RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"),
                "Walk direction changed, restarting the gait cycle");
    HexapodBridge::cancelLegActions();
    HexapodBridge::startWalkCycle(want_reversed);
  }

  void exit() override {
    RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Exiting Walking state");
  }
};
#pragma endregion States

using HexapodFsm = tinyfsm::FsmList<StateMachine>;
