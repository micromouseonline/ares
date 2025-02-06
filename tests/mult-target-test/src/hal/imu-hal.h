#pragma once

class IMUHAL {
 public:
  virtual void initialize() = 0;  // Initialize the IMU
  virtual void readData() = 0;    // Read data from the IMU
  virtual float getRollAngle() = 0;
  virtual float getPitchAngle() = 0;
  virtual float getYawAngle() = 0;
  virtual float getRollRate() = 0;
  virtual float getPitchRate() = 0;
  virtual float getYawRate() = 0;
  virtual ~IMUHAL() = default;
};
