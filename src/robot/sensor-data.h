//
// Created by peter on 22/11/24.
//

#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <mutex>

struct SensorData {
  float lfs_value = 0;
  float lds_value = 0;
  float rds_value = 0;
  float rfs_value = 0;
};

#include <functional>
/// Returns a SensorData struct
using SensorDataCallback = std::function<SensorData(float, float, float)>;

class SensorDataClass {
 public:
  // Constructor
  SensorDataClass() : frontDistance(0.0f), leftDistance(0.0f), rightDistance(0.0f) {}

  // Setters
  void setFrontDistance(float distance) {
    std::lock_guard<std::mutex> lock(mMutex);
    frontDistance = distance;
  }

  void setLeftDistance(float distance) {
    std::lock_guard<std::mutex> lock(mMutex);
    leftDistance = distance;
  }

  void setRightDistance(float distance) {
    std::lock_guard<std::mutex> lock(mMutex);
    rightDistance = distance;
  }

  // Getters
  float getFrontDistance() const {
    std::lock_guard<std::mutex> lock(mMutex);
    return frontDistance;
  }

  float getLeftDistance() const {
    std::lock_guard<std::mutex> lock(mMutex);
    return leftDistance;
  }

  float getRightDistance() const {
    std::lock_guard<std::mutex> lock(mMutex);
    return rightDistance;
  }

 private:
  float frontDistance;
  float leftDistance;
  float rightDistance;

  mutable std::mutex mMutex;  // mutable to allow const getters to lock the mutex
};

#endif  // SENSORDATA_H
