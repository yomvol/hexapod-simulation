#include "gait_engine.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <optional>

namespace tests {

constexpr double kCycleDuration = 2.0;  // matches the node's GAIT_CYCLE_DURATION
constexpr int kLegCount = 6;
constexpr int kBakePoints = 100;  // matches the node's TRAJ_POINTS_PER_CYCLE

class GaitTest : public ::testing::Test {
 protected:
  GaitTest() : gait_engine_(std::make_unique<GaitEngine>(kCycleDuration)) {}

  // body-frame foot position for a leg at a given time; nullopt if the IK
  // failed for that point
  std::optional<Vec3> footPosition(int leg_id, double time) {
    Vec3 foot_pos;
    auto joint_angles =
        gait_engine_->getLegTrajectoryPoint(leg_id, time, foot_pos);
    if (!joint_angles) {
      return std::nullopt;
    }
    return foot_pos;
  }

  double distance(const Vec3& a, const Vec3& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
  }

  std::unique_ptr<GaitEngine> gait_engine_;
};

// must run first: everything below assumes the gait bakes cleanly
TEST_F(GaitTest, FullCycleBakeSucceedsForAllLegs) {
  for (int leg_id = 0; leg_id < kLegCount; ++leg_id) {
    for (int i = 0; i < kBakePoints; ++i) {
      double t = kCycleDuration * i / kBakePoints;
      auto pos = footPosition(leg_id, t);
      ASSERT_TRUE(pos.has_value())
          << "IK failed for leg " << leg_id << " at t=" << t;
      EXPECT_TRUE(std::isfinite(pos->x) && std::isfinite(pos->y) &&
                  std::isfinite(pos->z));
    }
  }
}

TEST_F(GaitTest, TripodGroupsMoveComplementary) {
  // the tripod phasing guard: legs from opposite tripods must alternate, so
  // the distance between one foot of each group varies over the cycle by
  // roughly the step height + sway; same-phase legs would keep it constant
  const int tripod_pairs[][2] = {{0, 1}, {0, 5}, {2, 3}};
  const int kSamples = 100;
  for (const auto& pair : tripod_pairs) {
    double min_dist = 1.0;
    double max_dist = 0.0;
    for (int i = 0; i < kSamples; ++i) {
      double t = kCycleDuration * i / kSamples;
      double dist =
          distance(footPosition(pair[0], t).value(), footPosition(pair[1], t).value());
      min_dist = std::min(min_dist, dist);
      max_dist = std::max(max_dist, dist);
    }
    EXPECT_GT(max_dist - min_dist, 0.01)
        << "legs " << pair[0] << " (tripod A) and " << pair[1]
        << " (tripod B) never separate; distance range " << min_dist << " .. "
        << max_dist;
  }
}

TEST_F(GaitTest, FootTrajectoryIsPeriodicOverGaitCycle) {
  for (int leg_id = 0; leg_id < kLegCount; ++leg_id) {
    Vec3 pos_at_start;
    Vec3 pos_at_end;
    auto joints_start =
        gait_engine_->getLegTrajectoryPoint(leg_id, 0.0, pos_at_start);
    auto joints_end =
        gait_engine_->getLegTrajectoryPoint(leg_id, kCycleDuration, pos_at_end);
    ASSERT_TRUE(joints_start.has_value() && joints_end.has_value())
        << "IK failed for leg " << leg_id << " at a cycle boundary";

    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR((*joints_start)[j], (*joints_end)[j], 1e-9)
          << "leg " << leg_id << " joint " << j << " not periodic";
    }
    EXPECT_NEAR(pos_at_start.x, pos_at_end.x, 1e-9);
    EXPECT_NEAR(pos_at_start.y, pos_at_end.y, 1e-9);
    EXPECT_NEAR(pos_at_start.z, pos_at_end.z, 1e-9);
  }
}

TEST_F(GaitTest, FootTrajectoryIsContinuous) {
  // no sampled foot may jump: a phase discontinuity would move a foot by a
  // large fraction of the stride (4 cm) within one sample step
  const int kSamples = 1000;
  const double kMaxStep = 2e-3;  // ~3x headroom over the fastest gait motion
  for (int leg_id = 0; leg_id < kLegCount; ++leg_id) {
    Vec3 prev = footPosition(leg_id, 0.0).value();
    for (int i = 1; i < kSamples; ++i) {
      double t = kCycleDuration * i / kSamples;
      Vec3 curr = footPosition(leg_id, t).value();
      double step = distance(prev, curr);
      ASSERT_LE(step, kMaxStep)
          << "leg " << leg_id << " foot jumped " << step << " m at t=" << t;
      prev = curr;
    }
  }
}

}  // namespace tests
