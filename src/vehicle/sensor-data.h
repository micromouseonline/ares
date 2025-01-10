//
// Created by peter on 22/11/24.
//

#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <cstdint>

struct VehicleInputs {
  float lfs_distance = 0;
  float lds_distance = 0;
  float rds_distance = 0;
  float rfs_distance = 0;
  float lfs_power = 0;
  float lds_power = 0;
  float rds_power = 0;
  float rfs_power = 0;

  int action = 0;
  uint8_t buttons = 0;
  uint8_t leds = 0;
};

#endif  // SENSORDATA_H
