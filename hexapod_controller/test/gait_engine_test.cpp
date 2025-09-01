#include "gait_engine.hpp"
#include <gtest/gtest.h>

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

  int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  }
}  // namespace tests