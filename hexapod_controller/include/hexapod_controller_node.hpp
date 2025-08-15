#ifndef HEXAPOD_CONTROLLER_NODE_HPP
#define HEXAPOD_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <chrono>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <array>
#include <memory>
#include <functional>

#include "gait_engine.hpp"
#include "tinyfsm.hpp"

using namespace std::chrono_literals;

#pragma region Finite State Machine
// Finite State Machine Events
struct WakeUpEvent : tinyfsm::Event {};
struct CommandVelocityEvent : tinyfsm::Event {
  double linear_x;
  double angular_z;
  CommandVelocityEvent(double lx = 0.0, double az = 0.0) : linear_x(lx), angular_z(az) {}
};

/* ----------- bridge (FSM -> ROS side effects) ----------- */
/* node will set these callbacks so FSM code can call them in transition actions */
struct HexapodBridge {
  static std::function<void()> sendStandPose;
  static std::function<void()> startWalkCycle;
  static std::function<void()> cancelLegActions;
};

class StateMachine : public tinyfsm::Fsm<StateMachine> {
public:
    void react(tinyfsm::Event const & e) {} // default empty reaction

    virtual void react(WakeUpEvent const & e) {}
    virtual void react(CommandVelocityEvent const & e) {}

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

    void react(WakeUpEvent const & e) override {
        RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Transitioning to Standing state");
        auto action = []() {
            HexapodBridge::sendStandPose();
        };
        transit<Standing>(action);
    }
};

class Standing : public StateMachine {
public:
    using StateMachine::react;

    void entry() override {
        RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Entering Standing state");
    }

    void react(CommandVelocityEvent const & e) override {
        if (std::abs(e.linear_x) > VELOCITY_DEADZONE) {
            RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Transitioning to Walking state");
            auto action = []() {
                HexapodBridge::startWalkCycle();
            };
            transit<Walking>(action);
        }
        else {
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

    void react(CommandVelocityEvent const & e) override {
        if (std::abs(e.linear_x) < VELOCITY_DEADZONE && std::abs(e.angular_z) < VELOCITY_DEADZONE) {
            RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Transitioning to Standing state");
            auto action = []() {
                HexapodBridge::cancelLegActions();
                HexapodBridge::sendStandPose();
            };
            transit<Standing>(action);
        }
    }

    void exit() override {
        RCLCPP_INFO(rclcpp::get_logger("hexapod_controller"), "Exiting Walking state");
    }
};
#pragma endregion States

class HexapodControllerNode : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  struct TrajectoryPoint {
      std::vector<double> joint_angles; // we have 3 joints per leg, but can't use std::array here due to the action interface requirements
      std::vector<double> velocities; // angular velocities for each joint rad/s
      rclcpp::Duration relative_time_from_start; // time from the start of the trajectory
      Vec3 leg_tip_position_global;

      TrajectoryPoint();
  };

  struct Leg{
    std::string name;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr leg_client;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    visualization_msgs::msg::Marker marker;
    trajectory_msgs::msg::JointTrajectory trajectory;
    bool leg_action_in_progress = false;
  };

  HexapodControllerNode();

private:
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<GaitEngine> gait_engine_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool leg_frames_initialized_ = false;
  std::chrono::seconds GAIT_CYCLE_DURATION{2}; // time it takes for one full step with swing and stance
  int TRAJ_POINTS_PER_CYCLE = 100;
  std::array<Leg, 6> legs_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr wake_srv_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  int num_of_legs_in_action_ = 0;

  void prepareLegTrajectories();
  void updateGaitCycle();
  void sendStandPose();
  void startWalkCycle();
  void cancelLegActions();
};

#endif // HEXAPOD_CONTROLLER_NODE_HPP
