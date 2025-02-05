#pragma once

#include <iostream>
#include "hal/imu-hal.h"

class IMUAres : public IMUHAL {
 public:
  void initialize() override {
    std::cout << "initialised the IMU for ARES\n";
    // Ares-specific initialization code
    // e.g., SPI initialization for Ares IMU
  }

  void readData() override {
    // Ares-specific code to read data from IMU
    // e.g., SPI communication to fetch sensor data
  }
};
