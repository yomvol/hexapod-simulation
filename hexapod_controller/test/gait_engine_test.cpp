#include "gait_engine.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <stdexcept>

namespace tests {

  class GaitEngineTest : public ::testing::Test {
  protected:
    GaitEngineTest() {
      gait_engine_ = std::make_unique<GaitEngine>(1);
    }
    ~GaitEngineTest() override = default;

    std::unique_ptr<GaitEngine> gait_engine_;
  };

  TEST_F(GaitEngineTest, ForwardKinematicsInitPose) {
    ASSERT_NE(gait_engine_, nullptr);
    std::array<double, 3> input = {0.0, 0.0, 0.0};
    Vec3 result = gait_engine_->computeLegFK(input);
    EXPECT_NEAR(result.x, 0.1493, 1e-4); // expected coordinates of end point in coxa local frame
    EXPECT_NEAR(result.y, 0, 1e-4);
    EXPECT_NEAR(result.z, -0.18, 1e-4);
  }

  TEST_F(GaitEngineTest, ForwardKinematicsToInverseKinematics) {
    std::array<double, 3> input = {0.0, 0.0, 0.0};

    // it's pretty pointless to test other angle configurations, because IK solver won't come up with the same angles
    // for example, given the input {0.0, 0.0, M_PI / 6}, IK solver wants to bend femur, not tibia

    Vec3 fk_result = gait_engine_->computeLegFK(input);
    auto ik_result = gait_engine_->computeLegIK(fk_result);

    ASSERT_TRUE(ik_result.has_value());
    EXPECT_NEAR(ik_result->at(0), input[0], 1e-3);
    EXPECT_NEAR(ik_result->at(1), input[1], 1e-3);
    EXPECT_NEAR(ik_result->at(2), input[2], 1e-3);
  }

  TEST_F(GaitEngineTest, InverseKinematicsUnreachablePoint) {
    Vec3 unreachable_point = {10.0, 10.0, 10.0};  // clearly unreachable
    auto ik_result = gait_engine_->computeLegIK(unreachable_point);
    ASSERT_FALSE(ik_result.has_value());
  }

  TEST_F(GaitEngineTest, ConstructorRejectsNonPositiveCycleDuration) {
    // a zero or negative cycle duration would poison every gait phase with inf/NaN
    EXPECT_THROW(GaitEngine(0), std::invalid_argument);
    EXPECT_THROW(GaitEngine(-2), std::invalid_argument);
  }

  TEST_F(GaitEngineTest, FootTrajectoryIsPeriodicAcrossNegativeTime) {
    // fmod of a negative argument used to yield a negative phase, breaking the
    // stance/swing split for t < 0; the trajectory must wrap cleanly instead
    Vec3 at_zero, at_minus_cycle, at_minus_half, at_plus_half;
    auto j_zero = gait_engine_->getLegTrajectoryPoint(0, 0.0, at_zero);
    auto j_minus_cycle = gait_engine_->getLegTrajectoryPoint(0, -1.0, at_minus_cycle);
    auto j_minus_half = gait_engine_->getLegTrajectoryPoint(0, -0.5, at_minus_half);
    auto j_plus_half = gait_engine_->getLegTrajectoryPoint(0, 0.5, at_plus_half);
    ASSERT_TRUE(j_zero && j_minus_cycle && j_minus_half && j_plus_half);
    // a full cycle in the past lands on the same phase
    EXPECT_NEAR(at_minus_cycle.x, at_zero.x, 1e-9);
    EXPECT_NEAR(at_minus_cycle.y, at_zero.y, 1e-9);
    EXPECT_NEAR(at_minus_cycle.z, at_zero.z, 1e-9);
    // times half a cycle apart (in either direction) share a phase
    EXPECT_NEAR(at_minus_half.x, at_plus_half.x, 1e-9);
    EXPECT_NEAR(at_minus_half.y, at_plus_half.y, 1e-9);
    EXPECT_NEAR(at_minus_half.z, at_plus_half.z, 1e-9);
  }

  TEST_F(GaitEngineTest, InvalidLegIdThrows) {
    EXPECT_THROW(gait_engine_->getLegTrajectoryPoint(6, 0.0), std::invalid_argument);
    EXPECT_THROW(gait_engine_->getLegTrajectoryPoint(-1, 0.0), std::invalid_argument);
  }

  int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  }
}  // namespace tests