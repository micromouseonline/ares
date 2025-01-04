//
// Created by peter on 03/01/25.
//
#include <gtest/gtest.h>
#include <cfloat>
#include "common/pose.h"  // Adjust this path according to your project structure

// Test constructor and getters
TEST(PoseTest, 001_ConstructorAndGetters) {
  Pose pose(1.0f, 2.0f, 45.0f);
  EXPECT_FLOAT_EQ(pose.getX(), 1.0f);
  EXPECT_FLOAT_EQ(pose.getY(), 2.0f);
  EXPECT_FLOAT_EQ(pose.getTheta(), 45.0f);
  EXPECT_FLOAT_EQ(pose.getVelocity(), 0.0f);
  EXPECT_FLOAT_EQ(pose.getOmega(), 0.0f);
  EXPECT_FLOAT_EQ(pose.getDistance(), 0.0f);
}

// Test default constructor
TEST(PoseTest, 002_DefaultConstructor) {
  Pose pose;
  EXPECT_FLOAT_EQ(pose.getX(), 0.0f);
  EXPECT_FLOAT_EQ(pose.getY(), 0.0f);
  EXPECT_FLOAT_EQ(pose.getTheta(), 0.0f);
  EXPECT_FLOAT_EQ(pose.getVelocity(), 0.0f);
  EXPECT_FLOAT_EQ(pose.getOmega(), 0.0f);
  EXPECT_FLOAT_EQ(pose.getDistance(), 0.0f);
}

// Test setters
TEST(PoseTest, 010_Setters) {
  Pose pose;
  pose.setX(3.0f);
  pose.setY(4.0f);
  pose.setTheta(90.0f);
  pose.setVelocity(5.0f);
  pose.setOmega(1.0f);
  pose.setDistance(100.0f);

  EXPECT_FLOAT_EQ(pose.getX(), 3.0f);
  EXPECT_FLOAT_EQ(pose.getY(), 4.0f);
  EXPECT_FLOAT_EQ(pose.getTheta(), 90.0f);
  EXPECT_FLOAT_EQ(pose.getVelocity(), 5.0f);
  EXPECT_FLOAT_EQ(pose.getOmega(), 1.0f);
  EXPECT_FLOAT_EQ(pose.getDistance(), 100.0f);
}

// Test advance method
TEST(PoseTest, 020_Advance_Velocity) {
  Pose pose(0.0f, 0.0f, 0.0f);
  pose.setVelocity(1.0f);  // 1 unit per second
  pose.setOmega(0.0f);     // 90 degrees per second
  for (int i = 0; i < 1000; i++) {
    pose.advance(0.001f);  // Advance by 1 second
  }
  EXPECT_NEAR(pose.getDistance(), 1.0f, 0.005f);
  EXPECT_NEAR(pose.getTheta(), 0.0f, 0.005f);
  EXPECT_NEAR(pose.getX(), 1.0f, 0.0001f);
  EXPECT_NEAR(pose.getY(), 0.0f, 0.0001f);
}

// Test advance method
TEST(PoseTest, 021_Advance_Omega) {
  Pose pose(0.0f, 0.0f, 0.0f);
  pose.setVelocity(0.0f);  // 1 unit per second
  pose.setOmega(90.0f);    // 90 degrees per second
  for (int i = 0; i < 1000; i++) {
    pose.advance(0.001f);  // Advance by 1 second
  }
  EXPECT_NEAR(pose.getDistance(), 0.0f, 0.005f);
  EXPECT_NEAR(pose.getTheta(), 90.0f, 0.005f);
  EXPECT_NEAR(pose.getX(), 0.0f, 0.0001f);
  EXPECT_NEAR(pose.getY(), 0.0f, 0.0001f);
}

// Test advance method
TEST(PoseTest, 022_Advance_Both) {
  Pose pose(0.0f, 0.0f, 0.0f);
  pose.setVelocity(1.0f);  // 1 unit per second
  pose.setOmega(90.0f);    // 90 degrees per second
  for (int i = 0; i < 1000; i++) {
    pose.advance(0.001f);  // Advance by 1 second
  }
  EXPECT_NEAR(pose.getDistance(), 1.0f, 0.005f);
  EXPECT_NEAR(pose.getTheta(), 90.0f, 0.005f);
  EXPECT_NEAR(pose.getX(), 0.636119f, 0.0005f);
  EXPECT_NEAR(pose.getY(), 0.637119f, 0.0005f);
}

// Test angle wrapping
TEST(PoseTest, 030_AngleWrapping) {
  Pose pose(0.0f, 0.0f, 350.0f);
  pose.setOmega(20.0f);                     // 20 degrees per second
  pose.advance(1.0f);                       // Advance by 1 second
  EXPECT_FLOAT_EQ(pose.getTheta(), 10.0f);  // 350 + 20 = 370 % 360 = 10
}
