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

  int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  }
}  // namespace tests