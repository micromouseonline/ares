//
// Created by peter on 22/11/24.
//

#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <mutex>

struct SensorData {
  float lfs_distance = 0;
  float lds_distance = 0;
  float rds_distance = 0;
  float rfs_distance = 0;
  float lfs_power = 0;
  float lds_power = 0;
  float rds_power = 0;
  float rfs_power = 0;
};

#include <functional>
/// Returns a SensorData struct
using SensorDataCallback = std::function<SensorData(float, float, float)>;

class SensorDataClass {
 public:
  // Constructor
  SensorDataClass() : m_front_distance(0.0f), m_left_distance(0.0f), m_right_distance(0.0f) {}

  // Setters
  void setFrontDistance(float distance) {
    std::lock_guard<std::mutex> lock(mMutex);
    m_front_distance = distance;
  }

  void setLeftDistance(float distance) {
    std::lock_guard<std::mutex> lock(mMutex);
    m_left_distance = distance;
  }

  void setRightDistance(float distance) {
    std::lock_guard<std::mutex> lock(mMutex);
    m_right_distance = distance;
  }

  // Getters
  float getFrontDistance() const {
    std::lock_guard<std::mutex> lock(mMutex);
    return m_front_distance;
  }

  float getLeftDistance() const {
    std::lock_guard<std::mutex> lock(mMutex);
    return m_left_distance;
  }

  float getRightDistance() const {
    std::lock_guard<std::mutex> lock(mMutex);
    return m_right_distance;
  }

 private:
  float m_front_distance;
  float m_left_distance;
  float m_right_distance;

  mutable std::mutex mMutex;  // mutable to allow const getters to lock the mutex
};

#endif  // SENSORDATA_H
