//
// Created by peter on 29/11/24.
//

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <SFML/Graphics.hpp>

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
