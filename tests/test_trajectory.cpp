//
// Created by peter on 04/01/25.
//
#include <gtest/gtest.h>
#include "behaviour/trajectory.h"
#include "common/pose.h"

class TestTrajectory : public Trajectory {
  // A derived class to instantiate Trajectory for testing purposes
};

TEST(TrajectoryTest, 001_InitAndBegin) {
  TestTrajectory traj;
  EXPECT_TRUE(traj.isFinished());
  EXPECT_EQ(0.001f, traj.getDeltaTime());

  Pose p = traj.getCurrentPose();
  EXPECT_EQ(0, p.getX());
  EXPECT_EQ(0, p.getY());
  EXPECT_EQ(0, p.getTheta());
  EXPECT_EQ(0, traj.getCurrentStep());

  Pose startPose(1.0f, 2.0f, 30.0f);
  traj.init(startPose);
  p = traj.getCurrentPose();
  EXPECT_EQ(p.getX(), startPose.getX());
  EXPECT_EQ(p.getY(), startPose.getY());
  EXPECT_EQ(p.getTheta(), startPose.getTheta());

  traj.begin();
  EXPECT_FALSE(traj.isFinished());
}

TEST(TrajectoryTest, 002_DeltaTime) {
  TestTrajectory traj;
  traj.setDeltaTime(0.01f);
  EXPECT_FLOAT_EQ(traj.getDeltaTime(), 0.01f);
}

TEST(TrajectoryTest, 010_NextStep) {
  TestTrajectory traj;
  traj.begin();
  float initialVelocity = traj.next();
  EXPECT_FLOAT_EQ(initialVelocity, 10.0f);
  EXPECT_FALSE(traj.isFinished());
  for (int i = 0; i < 100; ++i) {
    traj.next();
  }
  EXPECT_EQ(100, traj.getCurrentStep());
  EXPECT_TRUE(traj.isFinished());
  traj.next();
  /// does not advance past end
  EXPECT_EQ(100, traj.getCurrentStep());
}

TEST(TrajectoryTest, 020_PoseAtTime) {
  TestTrajectory traj;
  Pose startPose(0.0f, 0.0f, 0.0f);
  traj.init(startPose);
  Pose poseAtTime = traj.getPoseAtTime(0.05f);
  EXPECT_NEAR(poseAtTime.getX(), 0.5f, 0.005f);
}

TEST(TrajectoryTest, 030_FinalPose) {
  TestTrajectory traj;
  Pose startPose(0.0f, 0.0f, 0.0f);
  traj.init(startPose);
  float duration = traj.get_duration();
  EXPECT_EQ(0.1f, duration);  /// 100 steps of 0.001 seconds
  Pose finalPose = traj.getFinalPose();
  EXPECT_TRUE(traj.isFinished());
  EXPECT_NEAR(finalPose.getX(), 2.0f, 0.005f);
}

TEST(TrajectoryTest, 040_Reset) {
  TestTrajectory traj;
  Pose startPose(1.0f, 2.0f, 30.0f);
  traj.init(startPose);
  traj.begin();
  traj.next();
  traj.reset();
  EXPECT_TRUE(traj.isFinished());
  EXPECT_EQ(0, traj.getCurrentStep());
  Pose pose = traj.getCurrentPose();
  EXPECT_FLOAT_EQ(pose.getX(), 1.0f);
  EXPECT_FLOAT_EQ(pose.getY(), 2.0f);
  EXPECT_FLOAT_EQ(pose.getTheta(), 30.0f);
}
