//
// Created by peter on 05/02/25.
//
#include <iostream>
#include "board-config.h"
#include "display.h"  // High-level interface for display
#include "imu.h"      // High-level interface for IMU
#include "vehicle/odometry.h"

int main() {
  initPeripherals();

  imu.readData();
  display.cls();

  std::cout << "IMU data read successfully." << std::endl;

  return 0;
}
