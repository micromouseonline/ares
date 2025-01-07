//
// Created by peter on 03/01/25.
//
#include <gtest/gtest.h>
#include "behaviour/maze.h"  // Assuming your Maze class is in this header

class MazeTest : public ::testing::Test {
 protected:
  Maze maze;

  void SetUp() override {
    // You can set up any shared resources here
    maze.initialise();
  }

  void TearDown() override {
    // Clean up any resources here if necessary
  }
};

// Test the initialization of the maze
TEST_F(MazeTest, 001_InitialiseMaze) {
  EXPECT_EQ(maze.getWidth(), 16);  // Default width is 16
  EXPECT_EQ(MASK_OPEN, maze.get_mask());
  maze.flood_manhattan(Location(7, 7));
  EXPECT_EQ(maze.cost(0, 0), 14);                               // Initially, cost should be the maximum value
  EXPECT_EQ(maze.wall_state(0, 0, DIR_N), WallState::UNKNOWN);  // Internal walls should be unknown
  EXPECT_EQ(maze.wall_state(0, 0, DIR_E), WallState::UNKNOWN);
  EXPECT_EQ(maze.wall_state(0, 0, DIR_S), WallState::WALL);
  EXPECT_EQ(maze.wall_state(0, 0, DIR_W), WallState::WALL);
}

// Test the setting and getting of the goal
TEST_F(MazeTest, 010_SetGoal) {
  maze.set_goal(5, 5);
  Location goal = maze.goal();
  EXPECT_EQ(goal.x, 5);
  EXPECT_EQ(goal.y, 5);
}

// Test the behavior of updating a wall state
TEST_F(MazeTest, 020_UpdateWallState) {
  maze.update_wall_state(0, 1, DIR_N, WallState::WALL);
  EXPECT_EQ(maze.wall_state(0, 1, DIR_N), WallState::WALL);
  maze.update_wall_state(0, 1, DIR_N, WallState::EXIT);
  EXPECT_EQ(maze.wall_state(0, 1, DIR_N), WallState::WALL);  // Wall state shouldn't change once set
}

// Test the behavior of has_unknown_walls
TEST_F(MazeTest, 030_HasUnknownWalls) {
  EXPECT_TRUE(maze.has_unknown_walls(0, 0));  // Should be true initially
  maze.update_wall_state(0, 0, DIR_N, WallState::WALL);
  maze.update_wall_state(0, 0, DIR_E, WallState::WALL);
  EXPECT_FALSE(maze.has_unknown_walls(0, 0));  // Should be false after updating the wall state
}

// Test the flood_manhattan method
TEST_F(MazeTest, 040_FloodManhattan) {
  maze.set_mask(MASK_OPEN);
  maze.flood_manhattan(maze.goal());
  EXPECT_LT(maze.cost(0, 0), UINT16_MAX);  // Cost should be updated for all cells
}

// Test the direction to smallest cost neighbor
TEST_F(MazeTest, 050_DirectionToSmallestCost) {
  maze.setCost(1, 0, 1);
  maze.setCost(0, 1, 2);
  maze.setCost(0, 0, 0);
  Location cell(0, 0);
  Direction start_heading = DIR_N;
  Direction best_direction = maze.direction_to_smallest(cell, start_heading);
  EXPECT_EQ(best_direction, DIR_E);  // The smallest cost is to the east
}

//// Test wall state isExit method
// TEST_F(MazeTest, IsExit) {
//   maze.set_wall_state(0, 0, DIR_N, WallState::EXIT);
//   EXPECT_TRUE(maze.is_exit(0, 0, DIR_N));   // Should return true as we set it to EXIT
//   EXPECT_FALSE(maze.is_exit(0, 0, DIR_E));  // Should return false for a direction with no exit
// }
//
//// Test setting and getting mask
// TEST_F(MazeTest, SetMask) {
//   maze.set_mask(Maze::MASK_CLOSED);
//   EXPECT_EQ(maze.get_mask(), Maze::MASK_CLOSED);
//   maze.set_mask(Maze::MASK_OPEN);
//   EXPECT_EQ(maze.get_mask(), Maze::MASK_OPEN);
// }
