//
// Created by peter on 29/11/24.
//

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

enum Activity {
  ACT_NONE,
  ACT_TEST_SS90,
  ACT_TEST_SS180,
  ACT_TEST_CIRCUIT,
  ACT_TEST_FOLLOW_TO,
  ACT_SEARCH,
};

enum Button {
  BTN_GO = (1 << 0),
  BTN_RESET = (1 << 1),
};

enum Led {
  LED_1 = (1 << 0),
  LED_2 = (1 << 1),
  LED_3 = (1 << 2),
  LED_4 = (1 << 3),
};

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

struct VehicleInputs {
  SensorData sensors;
  uint8_t buttons = 0;
};
/**
 * Positions here are in world coordinates with
 * the origin in the bottom left, x-axis to the right
 * Angles are with respect to the x-axis. Positive angles
 * are anti-clockwise
 *
 */
struct VehicleState {
  uint32_t ticks = 0;
  float x = 0;
  float y = 0;
  float angle = 0;
  float velocity = 0;
  float angular_velocity = 0;
  float total_distance = 0;  // accumulated from last reset
  uint8_t leds = 0;          // bitfield for led states
  uint8_t buttons = 0;       // bitfield for button states
  SensorData sensors;
};

#include <functional>
/// Returns a SensorData struct
using SensorDataCallback = std::function<VehicleInputs(VehicleState)>;

#endif  // ROBOT_STATE_H
