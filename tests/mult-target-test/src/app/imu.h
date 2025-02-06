#pragma once

#include "hal/imu-hal.h"

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

  float getHeading() {
    return imuHAL->getYawAngle();
  }

 private:
  IMUHAL* imuHAL;
};
