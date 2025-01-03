//
// Created by peter on 03/01/25.
//
#include <gtest/gtest.h>

TEST(ExampleTest, BasicAssertions) {
  EXPECT_EQ(1, 1);  // A simple test to check if GoogleTest is working
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
