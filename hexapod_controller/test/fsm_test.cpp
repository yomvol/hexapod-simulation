#include "fsm.hpp"

#include <gtest/gtest.h>

namespace tests {

// Drives the FSM through HexapodBridge callbacks only — no ROS graph, no
// executor. The bridge statics are the FSM's only side-effect surface, so
// counting their invocations fully characterizes each transition.
class FsmTest : public ::testing::Test {
 protected:
  void SetUp() override {
    stand_pose_calls_ = 0;
    walk_calls_ = 0;
    cancel_calls_ = 0;
    last_walk_reversed_ = false;

    HexapodBridge::sendStandPose = [this]() { ++stand_pose_calls_; };
    HexapodBridge::startWalkCycle = [this](bool is_reversed) {
      ++walk_calls_;
      last_walk_reversed_ = is_reversed;  // mirrors the node's is_walking_reversed_
    };
    HexapodBridge::cancelLegActions = [this]() { ++cancel_calls_; };
    HexapodBridge::isWalkingReversed = [this]() { return last_walk_reversed_; };

    HexapodFsm::start();  // forces the FSM back to Rest regardless of prior state
  }

  void StartForwardWalk() {
    HexapodFsm::dispatch(WakeUpEvent());
    HexapodFsm::dispatch(CommandVelocityEvent(0.5, 0.0));
    // reset the counters so walk-phase assertions only see new events
    stand_pose_calls_ = 0;
    walk_calls_ = 0;
    cancel_calls_ = 0;
  }

  int stand_pose_calls_;
  int walk_calls_;
  int cancel_calls_;
  bool last_walk_reversed_;
};

TEST_F(FsmTest, StartEntersRestState) {
  EXPECT_TRUE(StateMachine::is_in_state<Rest>());
}

TEST_F(FsmTest, WakeUpTransitionsToStandingAndSendsStandPose) {
  HexapodFsm::dispatch(WakeUpEvent());
  EXPECT_TRUE(StateMachine::is_in_state<Standing>());
  EXPECT_EQ(stand_pose_calls_, 1);
}

TEST_F(FsmTest, StandingIgnoresSubDeadzoneVelocity) {
  HexapodFsm::dispatch(WakeUpEvent());
  HexapodFsm::dispatch(CommandVelocityEvent(0.005, 0.0));
  EXPECT_TRUE(StateMachine::is_in_state<Standing>());
  EXPECT_EQ(walk_calls_, 0);
}

TEST_F(FsmTest, StandingStartsForwardWalkOnSignificantVelocity) {
  HexapodFsm::dispatch(WakeUpEvent());
  HexapodFsm::dispatch(CommandVelocityEvent(0.5, 0.0));
  EXPECT_TRUE(StateMachine::is_in_state<Walking>());
  EXPECT_EQ(walk_calls_, 1);
  EXPECT_FALSE(last_walk_reversed_);
}

TEST_F(FsmTest, StandingStartsReversedWalkOnNegativeVelocity) {
  HexapodFsm::dispatch(WakeUpEvent());
  HexapodFsm::dispatch(CommandVelocityEvent(-0.5, 0.0));
  EXPECT_TRUE(StateMachine::is_in_state<Walking>());
  EXPECT_EQ(walk_calls_, 1);
  EXPECT_TRUE(last_walk_reversed_);
}

TEST_F(FsmTest, WalkingHaltsOnZeroVelocityAndReturnsToStanding) {
  StartForwardWalk();
  HexapodFsm::dispatch(CommandVelocityEvent(0.0, 0.0));
  EXPECT_TRUE(StateMachine::is_in_state<Standing>());
  EXPECT_EQ(cancel_calls_, 1);
  EXPECT_EQ(stand_pose_calls_, 1);
}

TEST_F(FsmTest, WalkingIgnoresRepeatCommandInSameDirection) {
  StartForwardWalk();
  HexapodFsm::dispatch(CommandVelocityEvent(0.5, 0.0));
  EXPECT_TRUE(StateMachine::is_in_state<Walking>());
  EXPECT_EQ(cancel_calls_, 0);
  EXPECT_EQ(walk_calls_, 0);
}

TEST_F(FsmTest, WalkingIgnoresAngularOnlyCommand) {
  StartForwardWalk();
  HexapodFsm::dispatch(CommandVelocityEvent(0.0, 0.5));
  EXPECT_TRUE(StateMachine::is_in_state<Walking>());
  EXPECT_EQ(cancel_calls_, 0);
  EXPECT_EQ(walk_calls_, 0);
}

TEST_F(FsmTest, WalkingRestartsOnDirectionReversal) {
  StartForwardWalk();
  HexapodFsm::dispatch(CommandVelocityEvent(-0.5, 0.0));
  EXPECT_TRUE(StateMachine::is_in_state<Walking>());
  EXPECT_EQ(cancel_calls_, 1);
  EXPECT_EQ(walk_calls_, 1);
  EXPECT_TRUE(last_walk_reversed_);
}

}  // namespace tests
