#include "gait_engine.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace tests {

// link lengths from the engine's DH parameters, duplicated here so the tests
// can construct meaningful targets in the IK input frame (coxa-local, z below
// the coxa negative, origin on the ground)
constexpr double kCoxaLength = 0.052;
constexpr double kFemurLength = 0.06606;
constexpr double kTibiaLength = 0.16871;
constexpr double kMaxReach = kFemurLength + kTibiaLength;
constexpr double kMinReach = kTibiaLength - kFemurLength;

class IKTest : public ::testing::Test {
 protected:
  IKTest() { gait_engine_ = std::make_unique<GaitEngine>(1); }

  std::unique_ptr<GaitEngine> gait_engine_;
};

TEST_F(IKTest, IKRoundTripMatchesFKAcrossJointSpace) {
  // FK -> IK -> FK must reproduce the same *position* for every configuration
  // in the visualizer's joint ranges, even where IK picks different angles
  const double kStep = 0.15;
  const double kEps = 1e-9;
  for (double q1 = -M_PI / 4; q1 <= M_PI / 4 + kEps; q1 += kStep) {
    for (double q2 = -M_PI / 2; q2 <= M_PI / 2 + kEps; q2 += kStep) {
      for (double q3 = -M_PI / 2; q3 <= M_PI / 2 + kEps; q3 += kStep) {
        Vec3 target = gait_engine_->computeLegFK({q1, q2, q3});
        // the IK models the leg in the vertical plane at the target's azimuth,
        // which is only the true plane while the tip stays in front of the
        // coxa axis; configurations folding the tip behind it (negative signed
        // radial) are outside the model's domain and are skipped here
        double signed_radial = target.x * std::cos(q1) + target.y * std::sin(q1);
        if (signed_radial < 0.01) {
          continue;
        }
        auto ik_result = gait_engine_->computeLegIK(target);
        ASSERT_TRUE(ik_result.has_value())
            << "no IK solution for joint angles (" << q1 << ", " << q2 << ", "
            << q3 << ")";

        for (double angle : *ik_result) {
          if (!std::isfinite(angle)) {
            ADD_FAILURE() << "non-finite joint angle for angles (" << q1
                          << ", " << q2 << ", " << q3 << ")";
            return;
          }
        }

        Vec3 round_trip = gait_engine_->computeLegFK(*ik_result);
        double pos_error =
            sqrt(pow(round_trip.x - target.x, 2) + pow(round_trip.y - target.y, 2) +
                 pow(round_trip.z - target.z, 2));
        if (pos_error > 1e-3) {
          ADD_FAILURE() << "round-trip position error " << pos_error
                        << " m for angles (" << q1 << ", " << q2 << ", " << q3
                        << ")";
          return;
        }
      }
    }
  }
}

TEST_F(IKTest, IKRejectsTargetsBeyondOuterReach) {
  // straight ahead and straight down beyond femur+tibia (y = 0 keeps this
  // independent of the radial-distance convention)
  Vec3 beyond_ahead = {kCoxaLength + kMaxReach + 0.05, 0.0, -0.18};
  Vec3 beyond_below = {kCoxaLength, 0.0, -0.18 - kMaxReach - 0.05};

  EXPECT_FALSE(gait_engine_->computeLegIK(beyond_ahead).has_value());
  EXPECT_FALSE(gait_engine_->computeLegIK(beyond_below).has_value());
}

TEST_F(IKTest, IKRejectsTargetsInsideInnerReach) {
  // closer to the femur joint than |tibia - femur|: no 2-link solution exists,
  // so this must be rejected, never answered with NaN angles. The femur joint
  // sits at ground height + Z_OFFSET, i.e. at fk z = 0.
  Vec3 near_femur = {kCoxaLength + 0.03, 0.0, 0.0};
  Vec3 at_femur_joint = {kCoxaLength, 0.0, 0.0};

  EXPECT_FALSE(gait_engine_->computeLegIK(near_femur).has_value());
  EXPECT_FALSE(gait_engine_->computeLegIK(at_femur_joint).has_value());
}

TEST_F(IKTest, IKNeverReturnsNaNOverTargetGrid) {
  // dense grid over the IK input frame: every answer must be either a clean
  // rejection or a finite solution — NaN angles are never acceptable
  const double kEps = 1e-9;
  for (double y : {0.0, 0.1, -0.1}) {
    for (double x = -0.35; x <= 0.35 + kEps; x += 0.025) {
      for (double z = -0.45; z <= -0.05 + kEps; z += 0.025) {
        auto ik_result = gait_engine_->computeLegIK({x, y, z});
        if (!ik_result.has_value()) {
          continue;
        }
        for (double angle : *ik_result) {
          if (!std::isfinite(angle)) {
            ADD_FAILURE() << "NaN angle for target (" << x << ", " << y << ", "
                          << z << ")";
            return;
          }
        }
      }
    }
  }
}

}  // namespace tests
