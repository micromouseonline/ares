//
// Created by peter on 04/01/25.
//
#include <gtest/gtest.h>
#include "behaviour/trajectories/spinturn.h"
#include "behaviour/trajectory.h"
#include "common/pose.h"

TEST(SpinturnTest, 001_InitAndBegin) {
  Spinturn spinturn;
  EXPECT_TRUE(spinturn.isFinished());
  EXPECT_EQ(0.001f, spinturn.getDeltaTime());

  Pose p = spinturn.getCurrentPose();
  EXPECT_EQ(0, p.getX());
  EXPECT_EQ(0, p.getY());
  EXPECT_EQ(0, p.getAngle());
  EXPECT_EQ(0, spinturn.getCurrentStep());

  Pose startPose(1.0f, 2.0f, 30.0f);
  spinturn.init(startPose);
  p = spinturn.getCurrentPose();
  EXPECT_EQ(p.getX(), startPose.getX());
  EXPECT_EQ(p.getY(), startPose.getY());
  EXPECT_EQ(p.getAngle(), startPose.getAngle());

  spinturn.begin();
  EXPECT_FALSE(spinturn.isFinished());
}

TEST(SpinturnTest, 002_DeltaTime) {
  Spinturn spinturn;
  spinturn.setDeltaTime(0.01f);
  EXPECT_FLOAT_EQ(spinturn.getDeltaTime(), 0.01f);
}

TEST(SpinturnTest, 003_InitWithArgs) {
  Spinturn spinturn(100.0, 0.0, 200.0, 0.0, 1000.0);
  EXPECT_FLOAT_EQ(spinturn.getDeltaTime(), 0.001f);
  Pose pose(0, 0, 0);
  spinturn.init(pose);
  Pose trajectory_pose = spinturn.getCurrentPose();
  EXPECT_FLOAT_EQ(trajectory_pose.getX(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getY(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getAngle(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getVelocity(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getOmega(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getDistance(), 0.0f);
}

TEST(SpinturnTest, 010_Update) {
  Spinturn spinturn(100.0, 0.0, 200.0, 0.0, 1000.0);
  Pose pose(0, 0, 0);
  spinturn.init(pose);
  spinturn.begin();
  Pose current_pose = spinturn.getCurrentPose();
  EXPECT_FLOAT_EQ(current_pose.getVelocity(), 0.0f);
  EXPECT_FALSE(spinturn.isFinished());
  float w_max = 0;
  while (!spinturn.isFinished()) {
    spinturn.update();
    EXPECT_LE(spinturn.getCurrentPose().getOmega(), 1.10f * 200.0f);
    EXPECT_NEAR(spinturn.getCurrentPose().getVelocity(), 0.0f, 0.005);
  }
  EXPECT_EQ(699, spinturn.getCurrentStep());
  current_pose = spinturn.getCurrentPose();
  EXPECT_NEAR(current_pose.getX(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getY(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getAngle(), 100.0f, 0.005);
  EXPECT_NEAR(current_pose.getVelocity(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getOmega(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getDistance(), 0.0f, 0.005);
}

TEST(SpinturnTest, 010_Update_Negative) {
  Spinturn spinturn(-100.0, 0.0, 200.0, 0.0, 1000.0);
  Pose pose(0, 0, 0);
  spinturn.init(pose);
  spinturn.begin();
  Pose current_pose = spinturn.getCurrentPose();
  EXPECT_FLOAT_EQ(current_pose.getVelocity(), 0.0f);
  EXPECT_FALSE(spinturn.isFinished());
  while (!spinturn.isFinished()) {
    spinturn.update();
  }
  EXPECT_EQ(699, spinturn.getCurrentStep());
  current_pose = spinturn.getCurrentPose();
  EXPECT_NEAR(current_pose.getX(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getY(), 0.0f, 0.005);
  /// Note: we normalise angles as 0..360
  EXPECT_NEAR(current_pose.getAngle(), 260.0f, 0.05);  /// angles are less accurate
  EXPECT_NEAR(current_pose.getVelocity(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getOmega(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getDistance(), 0.0f, 0.005);
}

TEST(SpinturnTest, 040_Reset) {
  Spinturn spinturn;
  Pose startPose(1.0f, 2.0f, 30.0f);
  spinturn.init(startPose);
  spinturn.begin();
  spinturn.update();
  spinturn.reset();
  EXPECT_TRUE(spinturn.isFinished());
  EXPECT_EQ(0, spinturn.getCurrentStep());
  Pose pose = spinturn.getCurrentPose();
  EXPECT_FLOAT_EQ(pose.getX(), 1.0f);
  EXPECT_FLOAT_EQ(pose.getY(), 2.0f);
  EXPECT_FLOAT_EQ(pose.getAngle(), 30.0f);
}
