//
// Created by peter on 04/01/25.
//
#include <gtest/gtest.h>
#include "behaviour/actions.h"
#include "behaviour/trajectories/cubic.h"
#include "behaviour/trajectories/cubic_parameters.h"
#include "behaviour/trajectory.h"
#include "common/pose.h"

TEST(CubicTrajectoryTest, 001_InitAndBegin) {
  Cubic cubic;
  EXPECT_TRUE(cubic.isFinished());
  EXPECT_EQ(0.001f, cubic.getDeltaTime());

  Pose p = cubic.getCurrentPose();
  EXPECT_EQ(0, p.getX());
  EXPECT_EQ(0, p.getY());
  EXPECT_EQ(0, p.getAngle());
  EXPECT_EQ(0, cubic.getCurrentStep());

  Pose startPose(1.0f, 2.0f, 30.0f);
  cubic.init(startPose);
  p = cubic.getCurrentPose();
  EXPECT_EQ(p.getX(), startPose.getX());
  EXPECT_EQ(p.getY(), startPose.getY());
  EXPECT_EQ(p.getAngle(), startPose.getAngle());

  cubic.begin();
  EXPECT_FALSE(cubic.isFinished());
}

TEST(CubicTrajectoryTest, 002_DeltaTime) {
  Cubic cubic;
  cubic.setDeltaTime(0.01f);
  EXPECT_FLOAT_EQ(cubic.getDeltaTime(), 0.01f);
}

TEST(CubicTrajectoryTest, 003_InitWithArgs) {
  Cubic cubic(150.0f, 90.0f, 100.0f);
  EXPECT_FLOAT_EQ(cubic.getDeltaTime(), 0.001f);
  Pose pose(0, 0, 0);
  cubic.init(pose);
  Pose trajectory_pose = cubic.getCurrentPose();
  EXPECT_FLOAT_EQ(trajectory_pose.getX(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getY(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getAngle(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getVelocity(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getOmega(), 0.0f);
  EXPECT_FLOAT_EQ(trajectory_pose.getDistance(), 0.0f);
}

TEST(CubicTrajectoryTest, 010_Update) {
  float speed = 100.0f;
  float length = 115.6f;
  Cubic cubic(length, 90.0f, speed);
  Pose pose(0, 0, 0);
  cubic.init(pose);
  cubic.begin();
  Pose current_pose = cubic.getCurrentPose();
  EXPECT_FLOAT_EQ(current_pose.getVelocity(), 0.0f);
  EXPECT_FALSE(cubic.isFinished());
  while (!cubic.isFinished()) {
    cubic.update();
  }
  EXPECT_EQ(int(length / (speed * cubic.getDeltaTime())), cubic.getCurrentStep());
  current_pose = cubic.getCurrentPose();
  EXPECT_NEAR(current_pose.getX(), 70.0f, 0.1);
  EXPECT_NEAR(current_pose.getY(), 70.0f, 0.1);
  EXPECT_NEAR(current_pose.getAngle(), 90.0f, 0.005);
  EXPECT_NEAR(current_pose.getVelocity(), speed, 0.005);
  EXPECT_NEAR(current_pose.getOmega(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getDistance(), 115.6f, 0.005);
}

TEST(CubicTrajectoryTest, 010_Update_Negative) {
  float speed = 100.0f;
  float length = 115.6f;
  Cubic cubic(length, -90.0f, speed);
  Pose pose(0, 0, 0);
  cubic.init(pose);
  cubic.begin();
  Pose current_pose = cubic.getCurrentPose();
  EXPECT_FLOAT_EQ(current_pose.getVelocity(), 0.0f);
  EXPECT_FALSE(cubic.isFinished());
  while (!cubic.isFinished()) {
    cubic.update();
  }
  EXPECT_EQ(int(length / (speed * cubic.getDeltaTime())), cubic.getCurrentStep());
  current_pose = cubic.getCurrentPose();
  EXPECT_NEAR(current_pose.getX(), 70.0f, 0.1);
  EXPECT_NEAR(current_pose.getY(), -70.0f, 0.1);
  EXPECT_NEAR(current_pose.getAngle(), 270.0f, 0.005);
  EXPECT_NEAR(current_pose.getVelocity(), 100.0f, 0.005);
  EXPECT_NEAR(current_pose.getOmega(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getDistance(), 115.6f, 0.005);
}

TEST(CubicTrajectoryTest, 040_Reset) {
  Cubic cubic;
  Pose startPose(1.0f, 2.0f, 30.0f);
  cubic.init(startPose);
  cubic.begin();
  cubic.update();
  cubic.reset();
  EXPECT_TRUE(cubic.isFinished());
  EXPECT_EQ(0, cubic.getCurrentStep());
  Pose pose = cubic.getCurrentPose();
  EXPECT_FLOAT_EQ(pose.getX(), 1.0f);
  EXPECT_FLOAT_EQ(pose.getY(), 2.0f);
  EXPECT_FLOAT_EQ(pose.getAngle(), 30.0f);
}

TEST(CubicTrajectoryTest, 100_SS90ER) {
  int id = SS90FR - OP_TURN_SMOOTH;
  CubicTurnParameters params = test_cubic_params[id];
  float length = params.length;
  float speed = params.speed_max;
  float angle = params.angle;
  Cubic cubic(length, angle, speed);
  Pose pose(0, 0, 0);
  cubic.init(pose);
  cubic.begin();
  while (!cubic.isFinished()) {
    cubic.update();
  }
  Pose current_pose = cubic.getCurrentPose();
  EXPECT_NEAR(current_pose.getX(), 118.0f, 0.1);
  EXPECT_NEAR(current_pose.getY(), -118.0f, 0.1);
  EXPECT_NEAR(current_pose.getAngle(), 360.0 - 90.0f, 0.05);
  EXPECT_NEAR(current_pose.getVelocity(), speed, 0.005);
  EXPECT_NEAR(current_pose.getOmega(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getDistance(), 195.0f, 0.005);
  EXPECT_NEAR(current_pose.getElapsedTime(), 0.195f, 0.005);
}

TEST(CubicTrajectoryTest, 100_SS180L) {
  int id = SS180L - OP_TURN_SMOOTH;
  CubicTurnParameters params = test_cubic_params[id];
  float length = params.length;
  float speed = params.speed_max;
  float angle = params.angle;
  Cubic cubic(length, angle, speed);
  Pose pose(0, 0, 0);
  cubic.init(pose);
  cubic.begin();
  while (!cubic.isFinished()) {
    cubic.update();
  }
  Pose current_pose = cubic.getCurrentPose();
  EXPECT_NEAR(current_pose.getX(), 0.0f, 0.5);
  EXPECT_NEAR(current_pose.getY(), 180.0f, 0.5);
  EXPECT_NEAR(current_pose.getAngle(), angle, 0.05);
  EXPECT_NEAR(current_pose.getVelocity(), speed, 0.005);
  EXPECT_NEAR(current_pose.getOmega(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getDistance(), length, 0.005);
  EXPECT_NEAR(current_pose.getElapsedTime(), length / speed, 0.005);
}

TEST(CubicTrajectoryTest, 100_DD90L) {
  int id = DD90L - OP_TURN_SMOOTH;
  CubicTurnParameters params = test_cubic_params[id];
  float length = params.length;
  float speed = params.speed_max;
  float angle = params.angle;
  Cubic cubic(length, angle, speed);
  Pose pose(0, 0, 0);
  cubic.init(pose);
  cubic.begin();
  while (!cubic.isFinished()) {
    cubic.update();
  }
  Pose current_pose = cubic.getCurrentPose();
  EXPECT_NEAR(current_pose.getX(), 75.6f, 0.5);
  EXPECT_NEAR(current_pose.getY(), 75.6f, 0.5);
  EXPECT_NEAR(current_pose.getAngle(), angle, 0.05);
  EXPECT_NEAR(current_pose.getVelocity(), speed, 0.005);
  EXPECT_NEAR(current_pose.getOmega(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getDistance(), length, 0.005);
  EXPECT_NEAR(current_pose.getElapsedTime(), length / speed, 0.005);
}

TEST(CubicTrajectoryTest, 100_SS90EL) {
  int id = SS90EL - OP_TURN_SMOOTH;
  CubicTurnParameters params = test_cubic_params[id];
  float length = params.length;
  float speed = params.speed_max;
  float angle = params.angle;
  Cubic cubic(length, angle, speed);
  Pose pose(0, 0, 0);
  cubic.init(pose);
  cubic.begin();
  while (!cubic.isFinished()) {
    cubic.update();
  }
  Pose current_pose = cubic.getCurrentPose();
  EXPECT_NEAR(current_pose.getX(), 70.0f, 0.5);
  EXPECT_NEAR(current_pose.getY(), 70.0f, 0.5);
  EXPECT_NEAR(current_pose.getAngle(), angle, 0.05);
  EXPECT_NEAR(current_pose.getVelocity(), speed, 0.005);
  EXPECT_NEAR(current_pose.getOmega(), 0.0f, 0.005);
  EXPECT_NEAR(current_pose.getDistance(), length, 0.005);
  EXPECT_NEAR(current_pose.getElapsedTime(), length / speed, 0.005);
}
