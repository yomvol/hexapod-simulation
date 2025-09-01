#include "gait_engine.hpp"

#include <gtest/gtest.h>

namespace tests {

TEST(HEXAPOD_CONTROLLER_TEST, LOL) {
  ASSERT_EQ(4, 2 + 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace tests
