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

struct VehicleInputs {
  float lfs_distance = 0;
  float lds_distance = 0;
  float rds_distance = 0;
  float rfs_distance = 0;
  float lfs_power = 0;
  float lds_power = 0;
  float rds_power = 0;
  float rfs_power = 0;

  int activity = ACT_NONE;
  int activity_args = 0;
  uint8_t buttons = 0;
  uint8_t leds = 0;
};
/**
 * Positions here are in world coordinates with
 * the origin in the bottom left, x-axis to the right
 * Angles are with respect to the x-axis. Positive angles
 * are anti-clockwise
 *
 */
struct VehicleState {
  uint32_t ticks;
  float x;
  float y;
  float angle;
  float velocity;
  float omega;
  float total_distance;  // accumulated from last reset
  float move_distance;   // accumulated from last movement start
  float move_angle;      // accumulated from last movement start
  float cell_offset;     // distance through cell from border
  uint8_t leds;          // bitfield for led states
  uint8_t buttons;       // bitfield for button states
  bool activity_complete;
  //  VehicleInputs vehicle_inputs;

  VehicleState()
      : x(0.0f),
        y(0.0f),
        angle(0.0f),
        velocity(0.0f),  //
        omega(0.0f),     //
        total_distance(0.0f),
        move_distance(0.0f),
        move_angle(0.0f),
        cell_offset(0.0f),
        leds(0),
        buttons(0),
        activity_complete(true)
  //        vehicle_inputs()
  {
    //
  }
};

#include <functional>
/// Returns a SensorData struct
using SensorDataCallback = std::function<VehicleInputs(VehicleState)>;

#endif  // ROBOT_STATE_H
