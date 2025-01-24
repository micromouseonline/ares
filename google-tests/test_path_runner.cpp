//
// Created by peter on 04/01/25.
//
#include <gtest/gtest.h>
#include "behaviour/actions.h"
#include "behaviour/path-printer.h"
#include "behaviour/path-runner.h"
#include "behaviour/trajectory.h"
#include "common/pose.h"

class TrajectoryPathTest : public ::testing::Test {
 protected:
  PathRunner pathRunner;
  void SetUp() override {
    // You can set up any shared resources here
  }

  void TearDown() override {
    // Clean up any resources here if necessary
  }
};

TEST_F(TrajectoryPathTest, 001_empty_path_with_end) {
  /// all paths MUST have an end.
  const uint8_t path_actions[] = {ACT_END};
  Pose start_pose;
  Pose exit_pose = pathRunner.executeActionList(path_actions, start_pose);
  EXPECT_EQ(exit_pose.getX(), -999);
}

TEST_F(TrajectoryPathTest, 002_empty_path_with_begin) {
  const uint8_t path_actions[] = {ACT_BEGIN, ACT_END};
  Pose start_pose;
  start_pose.setSpeeds(100, 200);
  Pose exit_pose = pathRunner.executeActionList(path_actions, start_pose);
  EXPECT_EQ(exit_pose.getX(), start_pose.getX());
  EXPECT_EQ(exit_pose.getY(), start_pose.getY());
  EXPECT_EQ(exit_pose.getDistance(), start_pose.getDistance());
  EXPECT_EQ(exit_pose.getVelocity(), start_pose.getVelocity());
  EXPECT_EQ(exit_pose.getOmega(), start_pose.getOmega());
  EXPECT_EQ(exit_pose.getElapsedTime(), 0.0f);
}

TEST_F(TrajectoryPathTest, 005_single_straight) {
  const uint8_t path_actions[] = {ACT_BEGIN, FWD10, ACT_END};
  Pose start_pose;
  Pose exit_pose = pathRunner.executeActionList(path_actions, start_pose);
  EXPECT_NEAR(exit_pose.getX(), 1800.0f, 0.5f);
  EXPECT_EQ(exit_pose.getY(), start_pose.getY());
  EXPECT_NEAR(exit_pose.getDistance(), 1800.0f, 0.5f);
  EXPECT_EQ(exit_pose.getAngle(), start_pose.getAngle());
  EXPECT_EQ(exit_pose.getVelocity(), start_pose.getVelocity());
  EXPECT_EQ(exit_pose.getOmega(), start_pose.getOmega());
  EXPECT_NEAR(exit_pose.getElapsedTime(), 18.1f, 0.05f);
}

TEST_F(TrajectoryPathTest, 006_BSRSE) {
  //  const uint8_t path_actions[] = {ACT_BEGIN, FWD10, SS90FR, FWD10, ACT_END};
  //  Pose start_pose;
  //  Pose exit_pose = pathRunner.executeActionList(path_actions, start_pose);
  //  //  EXPECT_NEAR(exit_pose.getX(), 1800.0f, 0.5f);
  //  EXPECT_EQ(exit_pose.getY(), start_pose.getY());
  //  EXPECT_NEAR(exit_pose.getDistance(), 1800.0f, 0.5f);
  //  EXPECT_EQ(exit_pose.getAngle(), start_pose.getAngle());
  //  EXPECT_EQ(exit_pose.getVelocity(), start_pose.getVelocity());
  //  EXPECT_EQ(exit_pose.getOmega(), start_pose.getOmega());
}
