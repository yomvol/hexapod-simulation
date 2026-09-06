#pragma once
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <chrono>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "fsm.hpp"
#include "gait_engine.hpp"

using namespace std::chrono_literals;

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

  struct Leg {
    std::string name;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr leg_client;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    visualization_msgs::msg::Marker marker;
    trajectory_msgs::msg::JointTrajectory trajectory;
    bool leg_action_in_progress = false;
    rclcpp::Time action_start_time;
    GoalHandle::SharedPtr active_goal_handle;  // set on acceptance, cleared on terminal result
    bool counted_in_cycle = false;  // guards against double-counting a goal in num_of_legs_in_action_
  };

  HexapodControllerNode();

 private:
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr node_discovery_timer_;
  rclcpp::TimerBase::SharedPtr cmd_vel_watchdog_timer_;
  rclcpp::Time last_cmd_vel_time_;  // stamp of the last received /cmd_vel
  std::shared_ptr<GaitEngine> gait_engine_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool leg_frames_initialized_ = false;
  bool is_ready_to_stand_ = false;
  bool is_walking_ = false;
  bool is_walking_reversed_ = false;
  std::chrono::seconds GAIT_CYCLE_DURATION{2}; // time it takes for one full step with swing and stance
  // lead-in segment at the start of every walk cycle: carries the robot from
  // its current pose (stand pose, mid-gait pose after a reversal) to the gait's
  // phase-0 pose instead of demanding it within one 20 ms trajectory segment
  std::chrono::milliseconds WALK_BLEND_DURATION{500};
  // dead-man switch: halt the walk when no cmd_vel arrives for this long.
  // teleop streams commands while a key is held; a single tap walks for at
  // most this long
  std::chrono::milliseconds CMD_VEL_TIMEOUT{1000};
  int TRAJ_POINTS_PER_CYCLE = 100;
  double ERROR_TOLERANCE = 0.05; // error tolerance for leg actions
  std::array<Leg, 6> legs_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr wake_srv_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  int num_of_legs_in_action_ = 0;
  int rest_pose_completions_ = 0;  // rest-pose goals finished successfully; readiness requires all six
  uint64_t walk_cycle_id_ = 0;  // bumped on every (re)start and cancel; stale action callbacks bail on mismatch
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

  void handleNodeDiscovery();
  void sendRestPose();
  void prepareLegTrajectories();
  void sendStandPose();
  void startWalkCycle(bool is_reversed);
  void onLegFinished(int leg_id);
  void cancelLegActions();
  void checkCmdVelTimeout();
};
