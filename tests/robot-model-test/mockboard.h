//
// Created by peter on 03/02/25.
//

#pragma once
#include <stdint.h>
#include <iostream>
#include "vehicle/hal/board-interface.h"

// MockBoard class for testing
class MockBoard : public BoardInterface {
 public:
  void init() override {
    std::cout << "[MOCK] BasicBoard initialising." << std::endl;
  }

  void update() override {
    std::cout << "[MOCK] BasicBoard updating." << std::endl;
  }

  void setMotorVoltage(float left, float right) override {
    lastLeftVolts = left;
    lastRightVolts = right;
    std::cout << "[MOCK] Motor volts set: Left=" << left << ", Right=" << right << std::endl;
  }

  float lastLeftVolts = 0;
  float lastRightVolts = 0;
};
