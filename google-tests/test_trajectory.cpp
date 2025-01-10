//
// Created by peter on 04/01/25.
//
#include <gtest/gtest.h>
#include "behaviour/trajectory.h"
#include "common/pose.h"

TEST(TrajectoryTest, 001_InitAndBegin) {
  TestTrajectory traj;
  EXPECT_TRUE(traj.isFinished());
  EXPECT_EQ(0.001f, traj.getDeltaTime());

  Pose p = traj.getCurrentPose();
  EXPECT_EQ(0, p.getX());
  EXPECT_EQ(0, p.getY());
  EXPECT_EQ(0, p.getAngle());
  EXPECT_EQ(0, traj.getCurrentStep());

  Pose startPose(1.0f, 2.0f, 30.0f);
  traj.init(startPose);
  p = traj.getCurrentPose();
  EXPECT_EQ(p.getX(), startPose.getX());
  EXPECT_EQ(p.getY(), startPose.getY());
  EXPECT_EQ(p.getAngle(), startPose.getAngle());

  traj.begin();
  EXPECT_FALSE(traj.isFinished());
}

TEST(TrajectoryTest, 002_DeltaTime) {
  TestTrajectory traj;
  traj.setDeltaTime(0.01f);
  EXPECT_FLOAT_EQ(traj.getDeltaTime(), 0.01f);
}

TEST(TrajectoryTest, 003_InitWithArgs) {
  TestTrajectory traj(1.0, 2.0);
  traj.setDeltaTime(0.01f);
  EXPECT_FLOAT_EQ(traj.getDeltaTime(), 0.01f);
}

TEST(TrajectoryTest, 010_NextStep) {
  TestTrajectory traj(10.0, 20.0);
  traj.begin();
  traj.update();
  EXPECT_FLOAT_EQ(traj.getCurrentPose().getVelocity(), 10.0f);
  EXPECT_FALSE(traj.isFinished());
  for (int i = 0; i < 100; ++i) {
    traj.update();
  }
  EXPECT_EQ(100, traj.getCurrentStep());
  EXPECT_TRUE(traj.isFinished());
  traj.update();
  /// does not advance past end
  EXPECT_EQ(100, traj.getCurrentStep());
}

TEST(TrajectoryTest, 040_Reset) {
  TestTrajectory traj;
  Pose startPose(1.0f, 2.0f, 30.0f);
  traj.init(startPose);
  traj.begin();
  traj.update();
  traj.reset();
  EXPECT_TRUE(traj.isFinished());
  EXPECT_EQ(0, traj.getCurrentStep());
  Pose pose = traj.getCurrentPose();
  EXPECT_FLOAT_EQ(pose.getX(), 1.0f);
  EXPECT_FLOAT_EQ(pose.getY(), 2.0f);
  EXPECT_FLOAT_EQ(pose.getAngle(), 30.0f);
}
