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
  sf::Vector2f pos{96.0f, 96.0f};
  float x;
  float y;
  float v;
  float theta;
  float omega;

  RobotState() : x(0.0f), y(0.0f), v(0.0f), theta(0.0f), omega(0.0f) {
    //
  }
};

#endif  // ROBOT_STATE_H
