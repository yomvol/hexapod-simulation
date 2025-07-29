#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <urdf/model.h>
#include <chrono>
#include <visualization_msgs/msg/marker.hpp>

#include "gait_engine.hpp"

using namespace std::chrono_literals;

class HexapodControllerNode : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  HexapodControllerNode() : Node("hexapod_controller"), 
                            gait_engine_(std::make_shared<GaitEngine>()),
                            tf_buffer_(this->get_clock()),
                            tf_listener_(tf_buffer_) {
    // action clients to the leg controllers
    leg1_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, "/leg1_controller/follow_joint_trajectory");
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&HexapodControllerNode::Update, this));

    InitializeLegFrames();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr leg1_client_;
  std::shared_ptr<GaitEngine> gait_engine_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  bool leg_frames_initialized_ = false;
  
  void InitializeLegFrames() {
    try {
      // wait for tf frames to be available
      rclcpp::sleep_for(std::chrono::seconds(1));
      
      auto body_to_coxa = tf_buffer_.lookupTransform("body", "coxa1_lf", tf2::TimePointZero);
      auto coxa_to_tibia = tf_buffer_.lookupTransform("coxa1_lf", "tibia1_lf", tf2::TimePointZero);
      
      // convert directly to Eigen::Isometry3d using tf2_eigen
      Eigen::Isometry3d body_to_coxa_eigen = tf2::transformToEigen(body_to_coxa);
      Eigen::Isometry3d coxa_to_tibia_eigen = tf2::transformToEigen(coxa_to_tibia);
      
      gait_engine_->SetLegFrames(body_to_coxa_eigen, coxa_to_tibia_eigen);
      leg_frames_initialized_ = true;
      
      RCLCPP_INFO(this->get_logger(), "Leg frames initialized successfully");
      
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get leg transforms: %s", ex.what());
      // retry after a short delay
      auto retry_timer = this->create_wall_timer(
          std::chrono::seconds(2), 
          std::bind(&HexapodControllerNode::InitializeLegFrames, this));
    }
  }

  void Update() {
    if (!leg_frames_initialized_) {
      RCLCPP_DEBUG(this->get_logger(), "Waiting for leg frames to be initialized...");
      return;
    }

    if (!leg1_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Leg1 controller not ready.");
      return;
    }

    // DEBUG: Log the leg endpoint position
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "body";
    marker.header.stamp = this->get_clock()->now();
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    Vec3 leg_endpoint = gait_engine_->GetLegEndpoint(1);
    RCLCPP_INFO(this->get_logger(), "Leg 1 endpoint position: x=%.2f, y=%.2f, z=%.2f",
                leg_endpoint.x, leg_endpoint.y, leg_endpoint.z);

    geometry_msgs::msg::PointStamped leg_tip_point;
    leg_tip_point.header.frame_id = "body";
    leg_tip_point.header.stamp = this->get_clock()->now();
    leg_tip_point.point.x = leg_endpoint.x;
    leg_tip_point.point.y = leg_endpoint.y;
    leg_tip_point.point.z = leg_endpoint.z;
    marker.points.push_back(leg_tip_point.point);
    marker_pub_->publish(marker);

    // get joint angles from the gait engine
    // auto joint_angles = gait_engine_->GetLegTrajectoryPoint(1, this->now().seconds());
    auto leg_tip_pos = gait_engine_->ComputeFootPosition(1, this->now().seconds());
    auto joint_angles = gait_engine_->GetLegTrajectoryPoint(leg_tip_pos);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {"joint_1_1", "joint_1_2", "joint_1_3"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_angles;

    // desired time from the trajectory start to arrive at this trajectory point.
    point.time_from_start = rclcpp::Duration(100ms);
    traj.points.push_back(point);

    // send as a goal
    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = traj;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [](const GoalHandle::WrappedResult &result) {
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(rclcpp::get_logger("HexapodControllerNode"), "Leg1 action failed.");
      }
    };

    leg1_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HexapodControllerNode>());
  rclcpp::shutdown();
  return 0;
}
