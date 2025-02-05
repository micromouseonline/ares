#pragma once

class IMUHAL {
 public:
  virtual void initialize() = 0;  // Initialize the IMU
  virtual void readData() = 0;    // Read data from the IMU

  virtual ~IMUHAL() = default;
};
