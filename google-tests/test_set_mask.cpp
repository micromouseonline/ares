//
// Created by peter on 03/01/25.
//
#include <gtest/gtest.h>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include "application/applog-manager.h"
#include "behaviour/maze.h"  // Include the Maze class header
#include "common/core.h"

// A helper function to run concurrent set_mask operations
void setMaskConcurrently(Maze& maze, MazeMask mask, std::atomic<bool>& success_flag) {
  try {
    maze.set_mask(mask);  // Set the mask on the given cell
  } catch (const std::exception& e) {
    success_flag = false;  // If any exception occurs, mark the test as failed
  }
}

class ConcurrentTest : public ::testing::Test {
 protected:
  Maze maze;  // Instance of the Maze class to be used in tests

  // Initialize the maze before each test
  void SetUp() override {
    maze.initialise();  // Ensure the maze is initialized before each test
  }

  // Cleanup after each test
  void TearDown() override {
    // Any necessary cleanup can be done here
  }
};

// Test to ensure that concurrent calls to set_mask are thread-safe
TEST_F(ConcurrentTest, ConcurrentSetMaskIsSafe) {
  const MazeMask mask1 = MASK_OPEN;    // First mask type
  const MazeMask mask2 = MASK_CLOSED;  // Second mask type

  const int num_threads = 100;  // Number of concurrent threads

  std::atomic<bool> success_flag(true);  // Flag to track if any failure occurs

  std::vector<std::thread> threads;

  // Launch multiple threads to concurrently call set_mask
  for (int i = 0; i < num_threads; ++i) {
    MazeMask mask = (i % 2 == 0) ? mask1 : mask2;  // Alternate between two mask types
    threads.push_back(std::thread(setMaskConcurrently, std::ref(maze), mask, std::ref(success_flag)));
  }
  // Join all threads
  for (auto& t : threads) {
    t.join();
  }
  // After all threads finish, check if the test was successful
  ASSERT_TRUE(success_flag.load()) << "An exception occurred during concurrent set_mask calls.";

  // Verify that the final mask is either one of the expected values
  MazeMask final_mask = maze.get_mask();  // Assume you have a get_mask method to read the mask
  ASSERT_TRUE(final_mask == mask1 || final_mask == mask2) << "Final mask value is inconsistent: " << final_mask;
}

// Test the direction to smallest cost neighbor (ChatGPT generated)
TEST_F(ConcurrentTest, 050_DirectionToSmallestCost) {
  // Set up the maze with initial costs
  maze.setCost(1, 0, 1);
  maze.setCost(0, 1, 2);
  maze.setCost(0, 0, 0);

  // Create a vector of threads
  std::vector<std::thread> threads;

  // Atomic flag to control when the test should stop
  std::atomic<bool> test_running(true);

  // Function to simulate concurrent reads and writes
  auto test_direction = [this, &test_running]() {
    while (test_running) {
      // Perform a read and write operation on the maze
      Location cell(0, 0);
      Direction start_heading = DIR_N;
      // Read operation: Get the best direction
      Direction best_direction = maze.direction_to_smallest(cell, start_heading);
      EXPECT_EQ(best_direction, DIR_E);  // The smallest cost is to the east
      // Write operation: Modify the maze's cost
      maze.setCost(0, 1, 3);  // Change the cost to test concurrent writing
    }
  };

  // Launch multiple threads to concurrently run the test
  const int num_threads = 10;
  for (int i = 0; i < num_threads; ++i) {
    threads.push_back(std::thread(test_direction));
  }

  // Run the test for a short period and then stop
  std::this_thread::sleep_for(std::chrono::seconds(1));
  test_running = false;

  // Join all threads to ensure they complete before the test ends
  for (auto& t : threads) {
    t.join();
  }
}
