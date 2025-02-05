//
// Created by peter on 05/02/25.
//
#include <iostream>
#include "board-config.h"
#include "display/display.h"  // High-level interface for display
#include "imu/imu.h"          // High-level interface for IMU

int main() {
  initPeripherals();

  imu.readData();
  display.cls();

  std::cout << "IMU data read successfully." << std::endl;

  return 0;
}
