//
// Created by peter on 29/11/24.
//

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <SFML/Graphics.hpp>

/**
 * Positions here are in world coordinates with
 * the origin in the bottom left, x-axis to the right
 * Angles are with respect to the x-axis. Positive angles
 * are anti-clockwise
 *
 */
struct RobotState {
  uint32_t timestamp;
  float x;
  float y;
  float theta;
  float velocity;
  float omega;
  float total_distance;  // accumulated from last reset
  float cell_offset;     // distance through cell from border

  RobotState()
      : x(0.0f),
        y(0.0f),
        theta(0.0f),
        velocity(0.0f),  //
        omega(0.0f),     //
        total_distance(0.0f),
        cell_offset(0.0f) {
    //
  }
};

#endif  // ROBOT_STATE_H
