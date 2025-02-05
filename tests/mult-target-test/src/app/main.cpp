//
// Created by peter on 05/02/25.
//
#include <iostream>
#include "board-config.h"
#include "imu/imu.h"  // High-level interface for IMU

// Select the correct IMU driver depending on the target
#ifdef BOARD_ARES
IMUAres imuHal;
// DisplayOLED display;
#elif defined(BOARD_MR32)
IMUMR32 imuHal;
DisplayMatrix display;
#endif

int main() {
  initPeripherals();
  // Create IMU object with board-specific driver
  IMU imu(&imuHal);

  // Initialize and read data
  imu.initialize();
  imu.readData();
  //  display.initialize();
  //  display.cls();

  std::cout << "IMU data read successfully." << std::endl;

  return 0;
}
