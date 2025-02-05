#pragma once

#include "imu-hal.h"

class IMU {
 public:
  IMU(IMUHAL* hal)
      : imuHAL(hal) {
  }

  void initialize() {
    imuHAL->initialize();
  }

  void readData() {
    imuHAL->readData();
  }

 private:
  IMUHAL* imuHAL;
};
