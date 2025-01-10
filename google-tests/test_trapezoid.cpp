//
// Created by peter on 04/01/25.
//
#include <gtest/gtest.h>
#include "behaviour/trajectories/straight.h"
#include "behaviour/trajectory.h"
#include "common/pose.h"

TEST(TrapezoidTest, 001_InitAndBegin) {
  Straight trapezoid;
  EXPECT_TRUE(trapezoid.isFinished());
  EXPECT_EQ(0.001f, trapezoid.getDeltaTime());

  Pose p = trapezoid.getCurrentPose();
  EXPECT_EQ(0, p.getX());
  EXPECT_EQ(0, p.getY());
  EXPECT_EQ(0, p.getAngle());
  EXPECT_EQ(0, trapezoid.getCurrentStep());

  Pose startPose(1.0f, 2.0f, 30.0f);
  trapezoid.init(startPose);
  p = trapezoid.getCurrentPose();
  EXPECT_EQ(p.getX(), startPose.getX());
  EXPECT_EQ(p.getY(), startPose.getY());
  EXPECT_EQ(p.getAngle(), startPose.getAngle());

  trapezoid.begin();
  EXPECT_FALSE(trapezoid.isFinished());
}

TEST(TrapezoidTest, 002_DeltaTime) {
  Straight trapezoid;
  trapezoid.setDeltaTime(0.01f);
  EXPECT_FLOAT_EQ(trapezoid.getDeltaTime(), 0.01f);
}

TEST(TrapezoidTest, 003_InitWithArgs) {
  Straight trapezoid(100.0, 0.0, 200.0, 0.0, 1000.0);
  EXPECT_FLOAT_EQ(trapezoid.getDeltaTime(), 0.001f);
  Pose pose(0, 0, 0);
  trapezoid.init(pose);
  Pose trajectory_pose = trapezoid.getCurrentPose();
  EXPECT_FLOAT_EQ(trajectory_pose.getX(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getY(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getAngle(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getVelocity(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getOmega(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getDistance(), 0.0f);
}

TEST(TrapezoidTest, 010_Update) {
  Straight trapezoid(100.0, 0.0, 200.0, 0.0, 1000.0);
  Pose pose(0, 0, 0);
  trapezoid.init(pose);
  trapezoid.begin();
  Pose current_pose = trapezoid.getCurrentPose();
  EXPECT_FLOAT_EQ(current_pose.getVelocity(), 0.0f);
  EXPECT_FALSE(trapezoid.isFinished());
  while (!trapezoid.isFinished()) {
    trapezoid.update();
  }
  EXPECT_EQ(699, trapezoid.getCurrentStep());
  current_pose = trapezoid.getCurrentPose();
  EXPECT_NEAR(current_pose.getX(), 100.0f, 0.005);
  EXPECT_NEAR(current_pose.getY(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getAngle(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getVelocity(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getOmega(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getDistance(), 100.0f, 0.005);
}

TEST(TrapezoidTest, 010_Update_Negative) {
  Straight trapezoid(-100.0, 0.0, 200.0, 0.0, 1000.0);
  Pose pose(0, 0, 0);
  trapezoid.init(pose);
  trapezoid.begin();
  Pose current_pose = trapezoid.getCurrentPose();
  EXPECT_FLOAT_EQ(current_pose.getVelocity(), 0.0f);
  EXPECT_FALSE(trapezoid.isFinished());
  while (!trapezoid.isFinished()) {
    trapezoid.update();
  }
  EXPECT_EQ(699, trapezoid.getCurrentStep());
  current_pose = trapezoid.getCurrentPose();
  EXPECT_NEAR(current_pose.getX(), -100.0f, 0.005);
  EXPECT_NEAR(current_pose.getY(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getAngle(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getVelocity(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getOmega(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getDistance(), -100.0f, 0.005);
}

TEST(TrapezoidTest, 040_Reset) {
  Straight trapezoid;
  Pose startPose(1.0f, 2.0f, 30.0f);
  trapezoid.init(startPose);
  trapezoid.begin();
  trapezoid.update();
  trapezoid.reset();
  EXPECT_TRUE(trapezoid.isFinished());
  EXPECT_EQ(0, trapezoid.getCurrentStep());
  Pose pose = trapezoid.getCurrentPose();
  EXPECT_FLOAT_EQ(pose.getX(), 1.0f);
  EXPECT_FLOAT_EQ(pose.getY(), 2.0f);
  EXPECT_FLOAT_EQ(pose.getAngle(), 30.0f);
}
