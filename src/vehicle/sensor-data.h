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

#endif  // SENSORDATA_H
