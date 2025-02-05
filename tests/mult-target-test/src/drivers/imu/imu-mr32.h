#pragma once

#include <iostream>
#include "hal/imu-hal.h"

class IMUMR32 : public IMUHAL {
 public:
  void initialize() override {
    std::cout << "initialised the IMU for MR32\n";
    // MR32-specific initialization code
    // e.g., SPI initialization for MR32 IMU
  }

  void readData() override {
    // MR32-specific code to read data from IMU
    // e.g., SPI communication to fetch sensor data
  }
};
