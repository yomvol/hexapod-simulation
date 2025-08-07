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

#include "gait_engine.hpp"

using namespace std::chrono_literals;

class HexapodControllerNode : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  HexapodControllerNode() : Node("hexapod_controller"),
                            tf_buffer_(this->get_clock()),
                            tf_listener_(tf_buffer_) {
    // action clients to the leg controllers
    leg1_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, "/leg1_controller/follow_joint_trajectory");
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    marker_.header.frame_id = "body";
    marker_.header.stamp = this->get_clock()->now();
    marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_.scale.x = 0.005;
    marker_.scale.y = 0.005;
    marker_.scale.z = 0.005;
    marker_.color.r = 1.0f;
    marker_.color.g = 0.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0f;

    timer_ = this->create_wall_timer(GAIT_CYCLE_DURATION, std::bind(&HexapodControllerNode::updateGaitCycle, this));

    gait_engine_ = std::make_shared<GaitEngine>(GAIT_CYCLE_DURATION.count());

    // allowing tf buffer to populate
    init_timer_ = this->create_wall_timer(
        std::chrono::seconds(2), 
        std::bind(&HexapodControllerNode::initializeLegFrames, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr leg1_client_;
  std::shared_ptr<GaitEngine> gait_engine_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  bool leg_frames_initialized_ = false;
  bool leg1_action_in_progress_ = false;
  std::chrono::milliseconds GAIT_CYCLE_DURATION{10000}; // time it takes for one full step with swing and stance
  int TRAJ_POINTS_PER_CYCLE = 100;
  visualization_msgs::msg::Marker marker_;

  struct TrajectoryPoint {
    std::vector<double> joint_angles; // we have 3 joints per leg, but can't use std::array here
    // due to the action interface requirements
    std::vector<double> velocities; // angular velocities for each joint rad/s
    rclcpp::Duration relative_time_from_start; // time from the start of the trajectory
    Vec3 leg_tip_position_global;

    TrajectoryPoint() : joint_angles(3, 0.0), velocities(3, 0.0), relative_time_from_start(0, 0), leg_tip_position_global(0.0, 0.0, 0.0) {}
  };

  void initializeLegFrames() {
    try {      
      auto body_to_coxa = tf_buffer_.lookupTransform("body", "coxa1_lf", tf2::TimePointZero);
      auto coxa_to_tibia = tf_buffer_.lookupTransform("coxa1_lf", "tibia_lf", tf2::TimePointZero);
      
      Eigen::Isometry3d body_to_coxa_eigen = tf2::transformToEigen(body_to_coxa);
      Eigen::Isometry3d coxa_to_tibia_eigen = tf2::transformToEigen(coxa_to_tibia);
      
      gait_engine_->setLegFrames(body_to_coxa_eigen, coxa_to_tibia_eigen);
      leg_frames_initialized_ = true;
      
      RCLCPP_INFO(this->get_logger(), "Leg frames initialized successfully");
      
      if (init_timer_) {
        init_timer_->cancel();
        init_timer_ = nullptr;
      }
      
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get leg transforms: %s", ex.what());
    }
  }

  void updateGaitCycle() {
    if (!leg_frames_initialized_) {
      RCLCPP_DEBUG(this->get_logger(), "Waiting for leg frames to be initialized...");
      return;
    }

    if (!leg1_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Leg1 controller not ready.");
      return;
    }

    // Don't send new action if one is already in progress
    if (leg1_action_in_progress_) {
      RCLCPP_DEBUG(this->get_logger(), "Leg1 action still in progress, skipping new goal.");
      return;
    }

    marker_.points.clear();

    // create a complete trajectory for the entire gait cycle
    trajectory_msgs::msg::JointTrajectory traj;
    std::vector<TrajectoryPoint> trajectory_points;
    auto start_time = this->get_clock()->now();
    // if timestamp is not set, execution starts immediately
    // if timestamp is set, robot will freeze indefinitely REALLY WEIRD
    //traj.header.stamp = start_time;
    traj.joint_names = {"joint_1_1", "joint_1_2", "joint_1_3"};

    // first pass: generate all joint angles for the trajectory points
    for (int i = 0; i < TRAJ_POINTS_PER_CYCLE; ++i) {
      double time_ratio = static_cast<double>(i) / (TRAJ_POINTS_PER_CYCLE - 1);
      auto relative_time = rclcpp::Duration(std::chrono::milliseconds(static_cast<int64_t>(time_ratio * GAIT_CYCLE_DURATION.count())));

      Vec3 leg_tip_pos;
      auto joint_angles = gait_engine_->getLegTrajectoryPoint(1, (relative_time).seconds(), leg_tip_pos);

      if (!joint_angles) {
        RCLCPP_WARN(this->get_logger(), "Leg1 IK solution not found for trajectory point %d.", i);
        continue;
      }

      TrajectoryPoint point;
      point.relative_time_from_start = relative_time;
      point.joint_angles = joint_angles.value();
      point.leg_tip_position_global = leg_tip_pos;
      trajectory_points.push_back(point);

      RCLCPP_INFO(this->get_logger(), "Leg1 IK joint angles: theta1=%.2f, theta2=%.2f, theta3=%.2f",
                  joint_angles.value()[0] * 180.0 / M_PI, 
                  joint_angles.value()[1] * 180.0 / M_PI, 
                  joint_angles.value()[2] * 180.0 / M_PI);
    }

    double dt = (GAIT_CYCLE_DURATION.count() / 1000.0) / static_cast<double>(TRAJ_POINTS_PER_CYCLE - 1);

    // second pass: compute joint velocities and construct action message
    for (size_t i = 0; i < trajectory_points.size(); ++i) {
      if (trajectory_points.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Not enough trajectory points to compute velocities.");
        break;
      }
      auto& p = trajectory_points[i];

      if (i == 0) { // forward difference, first point
        for (int j = 0; j < 3; ++j) {
          p.velocities[j] = (trajectory_points[i + 1].joint_angles[j] - p.joint_angles[j]) / dt;
        }
      }
      else if (i == trajectory_points.size() - 1) { // last point should have zero velocities
        p.velocities = {0.0, 0.0, 0.0};
      }
      else { // central difference for all other points gives increased accuracy
        for (int j = 0; j < 3; ++j) {
          p.velocities[j] = (trajectory_points[i + 1].joint_angles[j] - trajectory_points[i - 1].joint_angles[j]) / (2 * dt);
        }
      }

      // RCLCPP_INFO(this->get_logger(), "Leg1 IK joint velocities: v1=%.2f, v2=%.2f, v3=%.2f",
      //             p.velocities[0] * 180.0 / M_PI, 
      //             p.velocities[1] * 180.0 / M_PI, 
      //             p.velocities[2] * 180.0 / M_PI);

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = p.joint_angles;
      point.velocities = p.velocities;

      // time from trajectory start - must be increasing
      point.time_from_start = p.relative_time_from_start;
      traj.points.push_back(point);

      geometry_msgs::msg::PointStamped leg_tip_point;
      leg_tip_point.header.frame_id = "body";
      leg_tip_point.header.stamp = this->get_clock()->now();
      leg_tip_point.point.x = p.leg_tip_position_global.x;
      leg_tip_point.point.y = p.leg_tip_position_global.y;
      leg_tip_point.point.z = p.leg_tip_position_global.z;
      marker_.points.push_back(leg_tip_point.point);

      RCLCPP_INFO(this->get_logger(), "Leg tip position at time %.2f: (%.2f, %.2f, %.2f)",
                  p.relative_time_from_start.seconds(),
                  p.leg_tip_position_global.x,
                  p.leg_tip_position_global.y,
                  p.leg_tip_position_global.z);
    }

    if (traj.points.empty()) {
      RCLCPP_WARN(this->get_logger(), "No valid trajectory points generated for leg1.");
      return;
    }

    marker_pub_->publish(marker_);
    
    // send as a goal
    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = traj;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandle::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        leg1_action_in_progress_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };
    
    send_goal_options.result_callback = [this](const GoalHandle::WrappedResult &result) {
      leg1_action_in_progress_ = false;
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(this->get_logger(), "Leg1 gait cycle action failed.");
        RCLCPP_WARN(this->get_logger(), "Reason: %d", static_cast<int>(result.code));
      } else {
        RCLCPP_INFO(this->get_logger(), "Leg1 gait cycle completed successfully.");
      }
    };

    leg1_action_in_progress_ = true;
    leg1_client_->async_send_goal(goal_msg, send_goal_options);
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HexapodControllerNode>());
  rclcpp::shutdown();
  return 0;
}
