#include "hexapod_controller_node.hpp"

FSM_INITIAL_STATE(StateMachine, Rest)
using MyFsmList = tinyfsm::FsmList<StateMachine>;
std::function<void()> HexapodBridge::sendStandPose = [](){};
std::function<void()> HexapodBridge::startWalkCycle = [](){};
std::function<void()> HexapodBridge::cancelLegActions = [](){};

HexapodControllerNode::HexapodControllerNode() : Node("hexapod_controller"),
                                                 tf_buffer_(this->get_clock()),
                                                 tf_listener_(tf_buffer_) {
    for (int leg_id = 0; leg_id < 6; ++leg_id) {
      auto leg_name = "leg" + std::to_string(leg_id + 1);

      legs_[leg_id].name = leg_name;
      legs_[leg_id].leg_client = rclcpp_action::create_client<FollowJointTrajectory>(this, leg_name + "_controller/follow_joint_trajectory");
      legs_[leg_id].marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/" + leg_name + "/visualization_marker", 10);
      legs_[leg_id].marker.header.frame_id = "body";
      legs_[leg_id].marker.ns = leg_name;
      //legs_[leg_id].marker.id = leg_id;
      legs_[leg_id].marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      //legs_[leg_id].marker.lifetime = rclcpp::Duration::from_seconds(1.0);
      legs_[leg_id].marker.scale.x = 0.005;
      legs_[leg_id].marker.scale.y = 0.005;
      legs_[leg_id].marker.scale.z = 0.005;
      legs_[leg_id].marker.color.r = 1.0f;
      legs_[leg_id].marker.color.g = 0.0f;
      legs_[leg_id].marker.color.b = 0.0f;
      legs_[leg_id].marker.color.a = 1.0f;
    }

    wake_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "wake_up", 
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            (void)request;
            RCLCPP_INFO(this->get_logger(), "Wake up service called");
            MyFsmList::dispatch(WakeUpEvent());
            response->success = true;
            response->message = "ANCIENT EVIL HAS AWOKEN!";
        });

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          MyFsmList::dispatch(CommandVelocityEvent(msg->linear.x, msg->angular.z));
        });
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          last_joint_state_ = msg;
        });

    HexapodBridge::sendStandPose = [this]() {this->sendStandPose();};
    HexapodBridge::startWalkCycle = [this]() {this->startWalkCycle();};
    HexapodBridge::cancelLegActions = [this]() {this->cancelLegActions();};
    MyFsmList::start();

    gait_engine_ = std::make_shared<GaitEngine>(GAIT_CYCLE_DURATION.count());

    // allowing tf buffer to populate
    init_timer_ = this->create_wall_timer(
        std::chrono::seconds(5), 
        std::bind(&HexapodControllerNode::prepareLegTrajectories, this));
  }

HexapodControllerNode::TrajectoryPoint::TrajectoryPoint() : joint_angles(3, 0.0), velocities(3, 0.0), relative_time_from_start(rclcpp::Duration::from_seconds(0)), leg_tip_position_global(0.0, 0.0, 0.0) {}

void HexapodControllerNode::prepareLegTrajectories() {
    for (int leg_id = 0; leg_id < 6; ++leg_id) {
      auto leg_name = legs_[leg_id].name;
      std::string suffix;
      switch (leg_id + 1) {
        case 1: suffix = "lf"; break; // left front
        case 2: suffix = "lm"; break; // left middle
        case 3: suffix = "lr"; break; // left rear
        case 4: suffix = "rr"; break; // right rear 
        case 5: suffix = "rm"; break; // right middle
        case 6: suffix = "rf"; break; // right front
      }
      auto leg_frame = "coxa1_" + suffix;

      try {
        auto body_to_coxa = tf_buffer_.lookupTransform("body", leg_frame, tf2::TimePointZero, std::chrono::seconds(10));
        Eigen::Isometry3d body_to_coxa_eigen = tf2::transformToEigen(body_to_coxa);
        gait_engine_->setLegFrames(leg_id, body_to_coxa_eigen);
      }
      catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get leg transforms: %s", ex.what());
        rclcpp::shutdown();
        return;
      }

      // prebake trajectories for each leg
      // creating a complete trajectory for the entire gait cycle
      trajectory_msgs::msg::JointTrajectory traj;
      std::vector<HexapodControllerNode::TrajectoryPoint> trajectory_points;
      // obviously, header stamp should be set later
      //traj.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1); // start after 100ms delay;
      std::string joint_name_prefix = "joint_" + std::to_string(leg_id + 1);
      traj.joint_names = {joint_name_prefix + "_1", joint_name_prefix + "_2", joint_name_prefix + "_3"};
      trajectory_points.reserve(TRAJ_POINTS_PER_CYCLE);

      // first pass: generate all joint angles for the trajectory points
      for (int i = 0; i < TRAJ_POINTS_PER_CYCLE; ++i) {
        double time_ratio = i / static_cast<double>(TRAJ_POINTS_PER_CYCLE);
        double time_sec = time_ratio * std::chrono::duration<double>(GAIT_CYCLE_DURATION).count();
        auto relative_time = rclcpp::Duration::from_seconds(time_sec);
        Vec3 leg_tip_pos;
        auto joint_angles = gait_engine_->getLegTrajectoryPoint(leg_id, relative_time.seconds(), leg_tip_pos);

        if (!joint_angles) {
          RCLCPP_WARN(this->get_logger(), (leg_name + " IK solution not found for trajectory point %d.").c_str(), i);
          continue;
        }

        TrajectoryPoint point;
        point.relative_time_from_start = relative_time;
        point.joint_angles = joint_angles.value();
        point.leg_tip_position_global = leg_tip_pos;
        trajectory_points.push_back(point);

        // RCLCPP_INFO(this->get_logger(), "%s IK joint angles at time %.2f: theta1=%.2f (%.2f), theta2=%.2f (%.2f), theta3=%.2f (%.2f)",
        //             leg_name.c_str(),
        //             relative_time.seconds(),
        //             joint_angles.value()[0],
        //             joint_angles.value()[0] * 180.0 / M_PI,
        //             joint_angles.value()[1], 
        //             joint_angles.value()[1] * 180.0 / M_PI,
        //             joint_angles.value()[2], 
        //             joint_angles.value()[2] * 180.0 / M_PI);
      }

      double dt = (GAIT_CYCLE_DURATION.count()) / static_cast<double>(TRAJ_POINTS_PER_CYCLE);

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

        // RCLCPP_INFO(this->get_logger(), leg_name + " IK joint velocities: v1=%.2f, v2=%.2f, v3=%.2f",
        //             p.velocities[0] * 180.0 / M_PI, 
        //             p.velocities[1] * 180.0 / M_PI, 
        //             p.velocities[2] * 180.0 / M_PI);

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = p.joint_angles;
        point.velocities = p.velocities;

        // time from trajectory start - must be increasing
        point.time_from_start = builtin_interfaces::msg::Duration();
        point.time_from_start.sec = static_cast<int32_t>(p.relative_time_from_start.seconds());
        point.time_from_start.nanosec = static_cast<uint32_t>((p.relative_time_from_start.nanoseconds() % 1000000000));
        traj.points.push_back(point);

        geometry_msgs::msg::PointStamped leg_tip_point;
        leg_tip_point.header.frame_id = "body";
        leg_tip_point.header.stamp = this->get_clock()->now();
        leg_tip_point.point.x = p.leg_tip_position_global.x;
        leg_tip_point.point.y = p.leg_tip_position_global.y;
        leg_tip_point.point.z = p.leg_tip_position_global.z;
        legs_[leg_id].marker.points.push_back(leg_tip_point.point);

        // RCLCPP_INFO(this->get_logger(), (leg_name + " tip position at time %.2f: (%.2f, %.2f, %.2f)").c_str(),
        //             p.relative_time_from_start.seconds(),
        //             p.leg_tip_position_global.x,
        //             p.leg_tip_position_global.y,
        //             p.leg_tip_position_global.z);
      }

      if (traj.points.empty()) {
        RCLCPP_WARN(this->get_logger(), ("No valid trajectory points generated for " + leg_name).c_str());
        return;
      }

      legs_[leg_id].trajectory = traj;
    }
    leg_frames_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Leg frames initialized, trajectories are ready");

    if (init_timer_) {
      init_timer_->cancel();
      init_timer_ = nullptr;
    }
  }

void HexapodControllerNode::sendStandPose() {
  for (auto & leg : legs_) {
    if (!leg.leg_action_in_progress) {
      if (!leg.leg_client->wait_for_action_server(std::chrono::milliseconds(500))) {
        RCLCPP_WARN(this->get_logger(), "%s action server not available", leg.name.c_str());
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Sending stand pose for leg %s", leg.name.c_str());

      FollowJointTrajectory::Goal goal_msg;
      trajectory_msgs::msg::JointTrajectory traj;
      traj.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1); // start after 100ms delay
      traj.joint_names = leg.trajectory.joint_names;
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.assign(traj.joint_names.size(), 0.0); // all zeros
      pt.velocities.assign(traj.joint_names.size(), 0.0);
      pt.time_from_start = rclcpp::Duration::from_seconds(1.0); // 1 second to reach the pose
      traj.points.push_back(pt);
      goal_msg.trajectory = traj;

      auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
      send_goal_options.goal_response_callback = [this](const GoalHandle::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
      };
      
      send_goal_options.result_callback = [this](const GoalHandle::WrappedResult &result) {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(this->get_logger(), "Leg action failed.");
          RCLCPP_WARN(this->get_logger(), "Reason: %d", static_cast<int>(result.code));
        } else {
          RCLCPP_INFO(this->get_logger(), "Leg action completed successfully.");
          // it's safe to assume that the stand pose is reached if one leg made it
          for (auto & leg : legs_) {
            leg.leg_action_in_progress = false;
          }
        }
      };

      leg.leg_action_in_progress = true;
      leg.leg_client->async_send_goal(goal_msg, send_goal_options);
    } else {
      RCLCPP_INFO(this->get_logger(), "Leg %s action already in progress, skipping stand pose.", leg.name.c_str());
    }
  }
}

void HexapodControllerNode::startWalkCycle() {
  if (!leg_frames_initialized_) {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for leg frames to be initialized...");
    return;
  }

  for (auto & leg : legs_) {
    if (!leg.leg_action_in_progress) {
      if (!leg.leg_client->wait_for_action_server(std::chrono::milliseconds(500))) {
        RCLCPP_WARN(this->get_logger(), "%s action server not available", leg.name.c_str());
        return;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Leg %s action already in progress, skipping new goal.", leg.name.c_str());
      return;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Starting walk cycle...");
  num_of_legs_in_action_ = 6;
  auto start_time = this->now() + 100ms; // start after 100ms delay
  auto joint_state = last_joint_state_;

  for (int leg_id = 0; leg_id < 6; ++leg_id) {
    auto & leg = legs_[leg_id];
    FollowJointTrajectory::Goal goal_msg;
    leg.marker_pub->publish(leg.marker); // publish marker for visualization
    leg.action_start_time = this->now();
    auto fresh_traj = leg.trajectory;
    fresh_traj.header.stamp = start_time;

    // we have to emplace the first point that coincides with the current joint state
    if (!fresh_traj.points.empty() && joint_state) {
      trajectory_msgs::msg::JointTrajectoryPoint first_point = fresh_traj.points.front();
      std::vector<double> sliced_joint_positions;
      std::copy(joint_state->position.begin() + leg_id * 3,
                joint_state->position.begin() + (leg_id + 1) * 3,
                std::back_inserter(sliced_joint_positions));
      first_point.positions = sliced_joint_positions; // use current joint state
      first_point.velocities.assign(3, 0.0);
      first_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
      fresh_traj.points[0] = first_point; // replace the first point
    }

    goal_msg.trajectory = fresh_traj;
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandle::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this, leg_id](GoalHandle::SharedPtr goal_handle, const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback) {
      auto time_from_start = this->now() - legs_[leg_id].action_start_time;

      if (time_from_start.seconds() >= GAIT_CYCLE_DURATION.count()) {
        RCLCPP_WARN(this->get_logger(), "Leg %d action timed out after %.2f seconds", leg_id + 1, time_from_start.seconds());
        legs_[leg_id].leg_action_in_progress = false;
        legs_[leg_id].leg_client->async_cancel_goal(goal_handle);

        num_of_legs_in_action_--;
        if (num_of_legs_in_action_ <= 0) {
          RCLCPP_INFO(this->get_logger(), "All legs have completed their actions.");
          for (auto & leg : legs_) {
            leg.leg_action_in_progress = false;
          }
          startWalkCycle();
        }
      }
      // if (feedback->error.positions[1] <= ERROR_TOLERANCE &&
      //     feedback->error.positions[2] <= ERROR_TOLERANCE) {
      //   legs_[leg_id].leg_action_in_progress = false;
      //   legs_[leg_id].leg_client->async_cancel_goal(goal_handle);
      // }
    };
    
    send_goal_options.result_callback = [this, leg_id](const GoalHandle::WrappedResult &result) {
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(this->get_logger(), "Leg action failed.");
        RCLCPP_WARN(this->get_logger(), "Reason: %d", static_cast<int>(result.code));
      } else {
        RCLCPP_INFO(this->get_logger(), "Leg action completed successfully.");
        rclcpp::Duration action_duration = this->now() - legs_[leg_id].action_start_time;
        RCLCPP_INFO(this->get_logger(), "Leg %d action duration: %.2f seconds", leg_id + 1, action_duration.seconds());
        num_of_legs_in_action_--;
        if (num_of_legs_in_action_ <= 0) {
          RCLCPP_INFO(this->get_logger(), "All legs have completed their actions.");
          for (auto & leg : legs_) {
            leg.leg_action_in_progress = false;
          }
          startWalkCycle();
        }
      }
    };

    leg.leg_action_in_progress = true;
    leg.leg_client->async_send_goal(goal_msg, send_goal_options);
  }

}

void HexapodControllerNode::cancelLegActions() {
    for (auto & leg : legs_) {
        if (leg.leg_action_in_progress) {
            RCLCPP_INFO(this->get_logger(), "Cancelling action for %s", leg.name.c_str());
            auto cancel_future = leg.leg_client->async_cancel_all_goals();
            if (cancel_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
                RCLCPP_INFO(this->get_logger(), "%s actions cancelled successfully", leg.name.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to cancel actions for %s", leg.name.c_str());
            }
            leg.leg_action_in_progress = false;
        }
    }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HexapodControllerNode>());
  rclcpp::shutdown();
  return 0;
}
