#include <gtest/gtest.h>
#include <iostream>
#include "mockboard.h"
#include "robot.h"  // Include Vehicle logic
#include "vehicle/hal/board-interface.h"

// Test fixture for Vehicle
class VehicleTest : public ::testing::Test {
 protected:
  MockBoard mockBoard;
  BasicBoard& basic = BasicBoard::getInstance();
  Vehicle* vehicle;  // Store singleton reference

  void SetUp() override {
    vehicle = &Vehicle::getInstance(&mockBoard);  // Assign the instance
  }
};

// Test: Check if setting speed correctly calculates motor voltage
TEST_F(VehicleTest, SetSpeed_CalculatesCorrectVoltage) {
  std::cout << "Voltage calculation correct" << std::endl;
  vehicle->setSpeed(10);  // Set speed to 10

  EXPECT_FLOAT_EQ(mockBoard.lastLeftVolts, 30.0f);
  EXPECT_FLOAT_EQ(mockBoard.lastRightVolts, 30.0f);
}

// Test: Singleton returns the same instance
TEST_F(VehicleTest, Singleton_ReturnsSameInstance) {
  std::cout << "Only one instance" << std::endl;
  Vehicle& instance1 = Vehicle::getInstance(&mockBoard);
  Vehicle& instance2 = Vehicle::getInstance();
  EXPECT_EQ(&instance1, &instance2);
}
// Test: Default Board is BasicBoard
TEST_F(VehicleTest, Singleton_ReturnsBasicBoardbyDefault) {
  std::cout << "BasicBoard is default" << std::endl;
  Vehicle& instance1 = Vehicle::getInstance();
  Vehicle& instance2 = Vehicle::getInstance(&BasicBoard::getInstance());
  EXPECT_EQ(&instance1, &instance2);
}
// Test: Default Board with and without parametersd
TEST_F(VehicleTest, Singleton_BasicBoardHasOptionalParameters) {
  std::cout << "BasicBoard is default" << std::endl;
  int number;
  int* parameter = &number;
  Vehicle& instance1 = Vehicle::getInstance(&BasicBoard::getInstance(parameter));
  Vehicle& instance2 = Vehicle::getInstance(&BasicBoard::getInstance());
  EXPECT_EQ(&instance1, &instance2);
}

// Main function for running tests
int main(int argc, char** argv) {
  std::cout << "Running tests..." << std::endl;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
