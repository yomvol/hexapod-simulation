#include "hexapod_controller_node.hpp"

HexapodControllerNode::HexapodControllerNode()
    : Node("hexapod_controller"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
  for (int leg_id = 0; leg_id < 6; ++leg_id) {
    auto leg_name = "leg" + std::to_string(leg_id + 1);

    legs_[leg_id].name = leg_name;
    legs_[leg_id].leg_client =
        rclcpp_action::create_client<FollowJointTrajectory>(
            this, leg_name + "_controller/follow_joint_trajectory");
    legs_[leg_id].marker_pub =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "/" + leg_name + "/visualization_marker", 10);
    legs_[leg_id].marker.header.frame_id = "body";
    legs_[leg_id].marker.ns = leg_name;
    legs_[leg_id].marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
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
        HexapodFsm::dispatch(WakeUpEvent());
        response->success = true;
        response->message = "ANCIENT EVIL HAS AWOKEN!";
      });

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_vel_time_ = this->now();
        HexapodFsm::dispatch(
            CommandVelocityEvent(msg->linear.x, msg->angular.z));
      });
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_joint_state_ = msg;
      });

  HexapodBridge::sendStandPose = [this]() { this->sendStandPose(); };
  HexapodBridge::startWalkCycle = [this](bool is_reversed) {
    is_walking_ = true;
    this->startWalkCycle(is_reversed);
  };
  HexapodBridge::cancelLegActions = [this]() { this->cancelLegActions(); };
  HexapodBridge::isWalkingReversed = [this]() { return is_walking_reversed_; };
  HexapodFsm::start();

  gait_engine_ = std::make_shared<GaitEngine>(GAIT_CYCLE_DURATION.count());

  is_ready_to_stand_ = false;
  last_cmd_vel_time_ = this->now();  // match the active clock type before any timer can fire
  node_discovery_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&HexapodControllerNode::handleNodeDiscovery, this));

  // allowing tf buffer to populate
  init_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&HexapodControllerNode::prepareLegTrajectories, this));

  cmd_vel_watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HexapodControllerNode::checkCmdVelTimeout, this));
}

// Dead-man switch: when commands stop arriving mid-walk (teleop crashed,
// publisher gone), halt through the normal FSM path instead of chaining gait
// cycles forever. Uses sim time consistently with the cmd_vel stamps.
void HexapodControllerNode::checkCmdVelTimeout() {
  if (!is_walking_) {
    return;
  }
  const double stale_for = (this->now() - last_cmd_vel_time_).seconds();
  if (stale_for <= CMD_VEL_TIMEOUT.count() / 1000.0) {
    return;
  }
  RCLCPP_WARN(this->get_logger(), "No cmd_vel for %.1f s — halting the walk", stale_for);
  HexapodFsm::dispatch(CommandVelocityEvent(0.0, 0.0));
}

HexapodControllerNode::TrajectoryPoint::TrajectoryPoint()
    : joint_angles(3, 0.0),
      velocities(3, 0.0),
      relative_time_from_start(rclcpp::Duration::from_seconds(0)),
      leg_tip_position_global(0.0, 0.0, 0.0) {}

void HexapodControllerNode::handleNodeDiscovery() {
  // Node-name matching is not a reliable readiness signal here: the
  // controller_manager spawners are named "spawner_legN_controller" and outlive
  // the window before the controllers themselves exist. Poll the action
  // servers directly — they only appear once each leg controller is active.
  for (auto& leg : legs_) {
    if (!leg.leg_client->action_server_is_ready()) {
      RCLCPP_DEBUG(this->get_logger(), "Waiting for %s action server...", leg.name.c_str());
      return;  // retry on the next timer tick
    }
  }

  RCLCPP_INFO(this->get_logger(), "All 6 leg action servers are ready.");
  node_discovery_timer_->cancel();
  sendRestPose();
}

void HexapodControllerNode::sendRestPose() {
  rest_pose_completions_ = 0;  // readiness requires every leg to succeed in this batch
  for (int leg_id = 0; leg_id < 6; ++leg_id) {
    auto& leg = legs_[leg_id];
    if (!leg.leg_action_in_progress) {
      if (!leg.leg_client->wait_for_action_server(
              std::chrono::milliseconds(500))) {
        RCLCPP_WARN(this->get_logger(), "%s action server not available", leg.name.c_str());
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Sending rest pose for leg %s", leg.name.c_str());

      FollowJointTrajectory::Goal goal_msg;
      trajectory_msgs::msg::JointTrajectory traj;
      traj.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);  // start after 100ms delay
      std::string joint_name_prefix = "joint_" + std::to_string(leg_id + 1);
      traj.joint_names = {joint_name_prefix + "_1", joint_name_prefix + "_2", joint_name_prefix + "_3"};
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.assign(traj.joint_names.size(), 0.0);  // all zeros
      pt.positions[1] = -0.835;
      pt.velocities.assign(traj.joint_names.size(), 0.0);
      pt.time_from_start = rclcpp::Duration::from_seconds(1.0);  // 1 second to reach the pose
      traj.points.push_back(pt);
      goal_msg.trajectory = traj;

      auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
      send_goal_options.goal_response_callback = [this](const GoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "Goal accepted by server, waiting for result");
        }
      };

      send_goal_options.result_callback = [this, leg_id](const GoalHandle::WrappedResult& result) {
        auto& leg = legs_[leg_id];
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(this->get_logger(), "%s action failed.",
                      leg.name.c_str());
          RCLCPP_WARN(this->get_logger(), "Reason: %d",
                      static_cast<int>(result.code));
          return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "%s action completed successfully.",
                    leg.name.c_str());
        leg.leg_action_in_progress = false;
        if (++rest_pose_completions_ == 6) {
          is_ready_to_stand_ = true;
          RCLCPP_INFO(this->get_logger(), "All legs reached the rest pose, ready to stand");
        }
      };

      leg.leg_action_in_progress = true;
      leg.leg_client->async_send_goal(goal_msg, send_goal_options);
    } else {
      RCLCPP_INFO(this->get_logger(), "%s action already in progress, skipping rest pose.", leg.name.c_str());
    }
  }
}

void HexapodControllerNode::prepareLegTrajectories() {
  // Bake everything into locals and commit only if every leg produced a full,
  // uniformly spaced cycle: the velocity pass divides by the constant grid dt,
  // so a single skipped (IK-unreachable) point would silently corrupt the
  // velocities around the gap. On any failure the previous trajectories are
  // kept and the init timer retries.
  std::array<trajectory_msgs::msg::JointTrajectory, 6> baked_trajs;
  std::array<std::vector<geometry_msgs::msg::Point>, 6> baked_markers;
  bool all_baked = true;

  for (int leg_id = 0; leg_id < 6; ++leg_id) {
    auto leg_name = legs_[leg_id].name;
    bool leg_failed = false;
    std::string suffix;
    switch (leg_id + 1) {
      case 1:
        suffix = "lf";
        break;  // left front
      case 2:
        suffix = "lm";
        break;  // left middle
      case 3:
        suffix = "lr";
        break;  // left rear
      case 4:
        suffix = "rr";
        break;  // right rear
      case 5:
        suffix = "rm";
        break;  // right middle
      case 6:
        suffix = "rf";
        break;  // right front
    }
    auto leg_frame = "coxa1_" + suffix;

    try {
      // Never wait for transforms inside a callback: the single-threaded
      // executor delivers /tf on this same thread, so a blocking wait could
      // only time out. Fail fast and let the init timer retry.
      auto body_to_coxa = tf_buffer_.lookupTransform(
          "body", leg_frame, tf2::TimePointZero);
      Eigen::Isometry3d body_to_coxa_eigen =
          tf2::transformToEigen(body_to_coxa);
      gait_engine_->setLegFrames(leg_id, body_to_coxa_eigen);
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(
          this->get_logger(), "Leg transforms not available yet (retrying): %s",
          ex.what());
      return;
    }

    // prebake trajectories for each leg
    // creating a complete trajectory for the entire gait cycle
    trajectory_msgs::msg::JointTrajectory traj;
    std::vector<HexapodControllerNode::TrajectoryPoint> trajectory_points;
    // obviously, header stamp should be set later like this traj.header.stamp = ...
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
        RCLCPP_ERROR(this->get_logger(), "%s: IK solution not found for trajectory point %d — bake aborted.",
                    leg_name.c_str(), i);
        leg_failed = true;
        break;
      }

      TrajectoryPoint point;
      point.relative_time_from_start = relative_time;
      point.joint_angles = joint_angles.value();
      point.leg_tip_position_global = leg_tip_pos;
      trajectory_points.push_back(point);

      // RCLCPP_INFO(this->get_logger(), "%s IK joint angles at time %.2f:
      // theta1=%.2f (%.2f), theta2=%.2f (%.2f), theta3=%.2f (%.2f)",
      //             leg_name.c_str(),
      //             relative_time.seconds(),
      //             joint_angles.value()[0],
      //             joint_angles.value()[0] * 180.0 / M_PI,
      //             joint_angles.value()[1],
      //             joint_angles.value()[1] * 180.0 / M_PI,
      //             joint_angles.value()[2],
      //             joint_angles.value()[2] * 180.0 / M_PI);
    }

    if (trajectory_points.size() != static_cast<size_t>(TRAJ_POINTS_PER_CYCLE)) {
      RCLCPP_ERROR(this->get_logger(), "%s: baked %zu of %d trajectory points — bake aborted.",
                  leg_name.c_str(), trajectory_points.size(), TRAJ_POINTS_PER_CYCLE);
      leg_failed = true;
    }

    double dt = (GAIT_CYCLE_DURATION.count()) / static_cast<double>(TRAJ_POINTS_PER_CYCLE);
    if (!leg_failed) {
      // belt and braces: the grid times are assigned by index above, so this
      // only trips if the time bookkeeping ever changes
      for (size_t i = 1; i < trajectory_points.size(); ++i) {
        auto delta = trajectory_points[i].relative_time_from_start -
                     trajectory_points[i - 1].relative_time_from_start;
        if (std::abs((delta - rclcpp::Duration::from_seconds(dt)).seconds()) > 1e-9) {
          RCLCPP_ERROR(this->get_logger(), "%s: non-uniform spacing at trajectory point %zu — bake aborted.",
                      leg_name.c_str(), i);
          leg_failed = true;
          break;
        }
      }
    }

    if (leg_failed) {
      all_baked = false;
      continue;  // keep this leg's previous trajectory
    }

    // second pass: compute joint velocities and construct action message
    for (size_t i = 0; i < trajectory_points.size(); ++i) {
      auto& p = trajectory_points[i];

      if (i == 0 || i == trajectory_points.size() - 1) {  // first and last points should have zero velocities
        p.velocities = {0.0, 0.0, 0.0};
      } else {
        // central difference for all other points gives increased accuracy
        for (int j = 0; j < 3; ++j) {
          p.velocities[j] = (trajectory_points[i + 1].joint_angles[j] -  trajectory_points[i - 1].joint_angles[j]) / (2 * dt);
        }
      }

      // RCLCPP_INFO(this->get_logger(), leg_name + " IK joint velocities:
      // v1=%.2f, v2=%.2f, v3=%.2f",
      //             p.velocities[0] * 180.0 / M_PI,
      //             p.velocities[1] * 180.0 / M_PI,
      //             p.velocities[2] * 180.0 / M_PI);

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = p.joint_angles;
      point.velocities = p.velocities;
      point.time_from_start = p.relative_time_from_start;  // time from trajectory start - must be increasing
      traj.points.push_back(point);

      geometry_msgs::msg::Point leg_tip_point;
      leg_tip_point.x = p.leg_tip_position_global.x;
      leg_tip_point.y = p.leg_tip_position_global.y;
      leg_tip_point.z = p.leg_tip_position_global.z;
      baked_markers[leg_id].push_back(leg_tip_point);

      // RCLCPP_INFO(this->get_logger(), (leg_name + " tip position at time
      // %.2f: (%.2f, %.2f, %.2f)").c_str(),
      //             p.relative_time_from_start.seconds(),
      //             p.leg_tip_position_global.x,
      //             p.leg_tip_position_global.y,
      //             p.leg_tip_position_global.z);
    }

    baked_trajs[leg_id] = traj;
  }

  if (!all_baked) {
    RCLCPP_ERROR(this->get_logger(),
                 "Trajectory bake incomplete — keeping previous trajectories; walking stays disabled.");
    return;  // the init timer keeps retrying
  }

  for (int leg_id = 0; leg_id < 6; ++leg_id) {
    legs_[leg_id].trajectory = baked_trajs[leg_id];
    legs_[leg_id].marker.points = baked_markers[leg_id];
  }
  leg_frames_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Leg frames initialized, trajectories are ready");

  if (init_timer_) {
    init_timer_->cancel();
    init_timer_ = nullptr;
  }
}

void HexapodControllerNode::sendStandPose() {
  if (!is_ready_to_stand_) {
    RCLCPP_WARN(this->get_logger(), "Hexapod is not ready to stand yet, ignoring stand pose command.");
    return;
  }
  if (!leg_frames_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Gait trajectories are not baked yet, ignoring stand pose command.");
    return;
  }

  for (int leg_id = 0; leg_id < 6; ++leg_id) {
    auto& leg = legs_[leg_id];
    if (!leg.leg_action_in_progress) {
      if (!leg.leg_client->wait_for_action_server(
              std::chrono::milliseconds(500))) { RCLCPP_WARN(this->get_logger(), "%s action server not available", leg.name.c_str());
        return;
      }

      // standing = the gait's phase-0 pose: standing and walking then share one
      // geometry, so there is no seam when a walk starts from standing (or when
      // a walk re-starts after a mid-cycle halt)
      auto stand_angles = gait_engine_->getLegTrajectoryPoint(leg_id, 0.0);
      if (!stand_angles) {
        RCLCPP_ERROR(this->get_logger(), "No IK solution for the stand pose of %s", leg.name.c_str());
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Sending stand pose for leg %s", leg.name.c_str());
      FollowJointTrajectory::Goal goal_msg;
      trajectory_msgs::msg::JointTrajectory traj;
      traj.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);  // start after 100ms delay
      traj.joint_names = leg.trajectory.joint_names;
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = stand_angles.value();
      pt.velocities.assign(traj.joint_names.size(), 0.0);
      pt.time_from_start = rclcpp::Duration::from_seconds(1.0);  // 1 second to reach the pose
      traj.points.push_back(pt);
      goal_msg.trajectory = traj;

      auto send_goal_options =
          rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
      send_goal_options.goal_response_callback = [this](const GoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "Goal accepted by server, waiting for result");
        }
      };

      send_goal_options.result_callback = [this, &leg](const GoalHandle::WrappedResult& result) {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(this->get_logger(), "%s action failed.",
                      leg.name.c_str());
          RCLCPP_WARN(this->get_logger(), "Reason: %d",
                      static_cast<int>(result.code));
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "%s action completed successfully.",
                      leg.name.c_str());
          leg.leg_action_in_progress = false;
        }
      };

      leg.leg_action_in_progress = true;
      leg.leg_client->async_send_goal(goal_msg, send_goal_options);
    } else {
      RCLCPP_INFO(this->get_logger(), "%s action already in progress, skipping stand pose.", leg.name.c_str());
    }
  }
}

// Single accounting path for a walk goal reaching a terminal state
// (success, abort, rejection, or watchdog cancel). Idempotent per goal.
void HexapodControllerNode::onLegFinished(int leg_id) {
  auto& leg = legs_[leg_id];
  if (leg.counted_in_cycle) {
    return;
  }
  leg.counted_in_cycle = true;
  leg.leg_action_in_progress = false;

  if (--num_of_legs_in_action_ > 0) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "All legs have completed their actions.");
  for (auto& l : legs_) {
    l.leg_action_in_progress = false;
  }
  if (is_walking_) {
    startWalkCycle(is_walking_reversed_);
  }
}

void HexapodControllerNode::startWalkCycle(bool is_reversed) {
  if (!leg_frames_initialized_) {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for leg frames to be initialized...");
    return;
  }

  is_walking_reversed_ = is_reversed;

  for (auto& leg : legs_) {
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
  const uint64_t cycle_id = ++walk_cycle_id_;  // invalidates callbacks of any previous cycle
  num_of_legs_in_action_ = 6;
  auto start_time = this->now() + 100ms;  // start after 100ms delay
  auto joint_state = last_joint_state_;

  for (int leg_id = 0; leg_id < 6; ++leg_id) {
    auto& leg = legs_[leg_id];
    leg.counted_in_cycle = false;
    FollowJointTrajectory::Goal goal_msg;
    leg.marker_pub->publish(leg.marker);  // publish marker for visualization
    leg.action_start_time = start_time;
    auto fresh_traj = leg.trajectory;
    if (is_reversed) {
      std::vector<trajectory_msgs::msg::JointTrajectoryPoint> reversed = std::vector<trajectory_msgs::msg::JointTrajectoryPoint>(
        fresh_traj.points.rbegin(), fresh_traj.points.rend());
      for (int i = 0; i < reversed.size(); ++i) {
        reversed[i].time_from_start = fresh_traj.points[i].time_from_start;
        // the copy above kept the forward-pass velocities, but the mirrored
        // points are traversed backwards in time — flip the sign so the
        // controller's velocity feed-forward pushes the right way
        for (auto& velocity : reversed[i].velocities) {
          velocity = -velocity;
        }
      }
      fresh_traj.points = reversed;
    }
    fresh_traj.header.stamp = start_time;

    // Insert the current joint state as a lead-in point at t = 0 and shift the
    // baked cycle by the blend duration. Replacing the phase-0 point instead
    // (the old behavior) meant the controller had to reach the phase-1 target
    // within one 20 ms segment — a violent lurch at every walk start.
    if (!fresh_traj.points.empty() && joint_state) {
      // resolve the leg's joints by name: positional slicing into the shared
      // /joint_states message silently breaks if the publisher's ordering ever
      // differs from the URDF declaration order
      std::vector<double> sliced_joint_positions;
      bool all_joints_found = true;
      for (const auto& joint_name : fresh_traj.joint_names) {
        auto it = std::find(joint_state->name.begin(), joint_state->name.end(), joint_name);
        auto index = std::distance(joint_state->name.begin(), it);
        if (it == joint_state->name.end() ||
            index >= static_cast<std::ptrdiff_t>(joint_state->position.size())) {
          all_joints_found = false;
          break;
        }
        sliced_joint_positions.push_back(joint_state->position[index]);
      }

      if (!all_joints_found) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "%s joints missing from /joint_states — starting from the baked phase-0 pose",
            leg.name.c_str());
      } else {
        const double blend_sec = WALK_BLEND_DURATION.count() / 1000.0;
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> blended;
        blended.reserve(fresh_traj.points.size() + 1);

        trajectory_msgs::msg::JointTrajectoryPoint current;
        current.positions = sliced_joint_positions;  // start from where the robot is
        current.velocities.assign(3, 0.0);
        current.time_from_start = rclcpp::Duration::from_seconds(0.0);
        blended.push_back(current);

        for (auto& pt : fresh_traj.points) {
          pt.time_from_start = rclcpp::Duration::from_seconds(
              rclcpp::Duration(pt.time_from_start).seconds() + blend_sec);
          blended.push_back(pt);
        }
        fresh_traj.points = blended;
      }
    }

    goal_msg.trajectory = fresh_traj;
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this, leg_id, cycle_id](const GoalHandle::SharedPtr& goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        onLegFinished(leg_id);
        return;
      }
      if (cycle_id != walk_cycle_id_) {
        // a cancel or restart raced with this acceptance — don't let it run
        legs_[leg_id].leg_client->async_cancel_goal(goal_handle);
        return;
      }
      legs_[leg_id].active_goal_handle = goal_handle;
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    };

    send_goal_options.feedback_callback = [this, leg_id, cycle_id](GoalHandle::SharedPtr goal_handle,
      const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback) {
      if (cycle_id != walk_cycle_id_) {
        return;  // feedback of a superseded cycle
      }
      auto time_from_start = this->now() - legs_[leg_id].action_start_time;

      // Walk goals that involve motion currently never deliver a result on
      // their own (JTC goal-termination issue), so this watchdog is the normal
      // per-cycle terminator, not an error path. It is tied to
      // GAIT_CYCLE_DURATION + WALK_BLEND_DURATION on purpose: the cycle
      // restarts exactly when the gait pattern ends, without a dead hold (the
      // baked points are shifted by the blend lead-in, so goal-local time to
      // the end of the pattern grows by the same amount). The goal's
      // trajectory spans [send + 0.1 s stamp, send + WALK_BLEND_DURATION +
      // GAIT_CYCLE_DURATION + 0.1 s], so the last ~0.1 s is cut — the foot is
      // within ~1-2 mm of the neutral stance pose there and the next cycle's
      // seam point re-anchors from the measured state.
      if (time_from_start.seconds() >=
          GAIT_CYCLE_DURATION.count() + WALK_BLEND_DURATION.count() / 1000.0) {
        RCLCPP_INFO(this->get_logger(), "Leg %d walk goal ended by watchdog after %.2f seconds",
                    leg_id + 1, time_from_start.seconds());
        legs_[leg_id].leg_client->async_cancel_goal(goal_handle);
        onLegFinished(leg_id);
      }
    };

    send_goal_options.result_callback = [this, leg_id, cycle_id](const GoalHandle::WrappedResult& result) {
      legs_[leg_id].active_goal_handle = nullptr;  // terminal in every code path
      if (cycle_id != walk_cycle_id_) {
        return;  // result of a superseded cycle (canceled or restarted)
      }

      if (result.code == rclcpp_action::ResultCode::CANCELED) {
        onLegFinished(leg_id);  // no-op if the watchdog already accounted for it
        return;
      }

      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(this->get_logger(), "Leg action failed.");
        RCLCPP_WARN(this->get_logger(), "Reason: %d", static_cast<int>(result.code));
      } else {
        RCLCPP_INFO(this->get_logger(), "Leg action completed successfully.");
        rclcpp::Duration action_duration = this->now() - legs_[leg_id].action_start_time;
        RCLCPP_INFO(this->get_logger(), "Leg %d action duration: %.2f seconds", leg_id + 1, action_duration.seconds());
      }
      onLegFinished(leg_id);
    };

    leg.leg_action_in_progress = true;
    leg.leg_client->async_send_goal(goal_msg, send_goal_options);
  }
}

void HexapodControllerNode::cancelLegActions() {
  is_walking_ = false;
  is_walking_reversed_ = false;
  ++walk_cycle_id_;  // feedback/result callbacks of the old cycle must not touch state anymore
  num_of_legs_in_action_ = 0;

  for (auto& leg : legs_) {
    if (leg.active_goal_handle) {
      leg.leg_client->async_cancel_goal(leg.active_goal_handle);
    }
    leg.active_goal_handle = nullptr;
    leg.leg_action_in_progress = false;
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HexapodControllerNode>());
  rclcpp::shutdown();
  return 0;
}
