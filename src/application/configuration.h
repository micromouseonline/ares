#pragma once
#include <SFML/Graphics.hpp>

#include <string>
#include "common/core.h"

namespace conf {

  // Debug flags

  // Maze Dimensions
  const float MazeSize = (16 * 180.0f) + 12.0f;  // 2892.0f;

  // Window configuration
  const std::string AppName = "ARES Simulator for Micromouse";
  const sf::Vector2u WindowSize = {1900, 1000};
  const sf::VideoMode VideoMode = {WindowSize.x, WindowSize.y, 32};
  const sf::Color WindowBackGround = sf::Color(16, 16, 16, 255);

  const sf::View MazeView({MazeSize / 2.0f, MazeSize / 2.0f}, {MazeSize, -MazeSize});

  const uint32_t FrameRate = 60;
  const float WindowPadding = 10.0f;
  const float MazeViewScreenSize = 964.0f;

  const sf::Color MazeGoalColour = sf::Color(30, 70, 30, 64);
  const sf::Color MazeUnseenColour = sf::Color(20, 20, 20, 64);
  const sf::Color MazeBaseColour = sf::Color(30, 50, 50, 64);
  const sf::Color PostColour = sf::Color(192, 0, 0);

  // Robot configuration defaults
  // TODO: Add the body details
  enum WallSensorName { LFS = 0, LDS, RDS, RFS, SENSOR_COUNT };
  const SensorGeometry SensorDefaultOffsets[SENSOR_COUNT] = {
      {.x = 25, .y = 30, .theta = 10, .halfAngle = 5.0f, .rayCount = 12},
      {.x = 50, .y = 10, .theta = 60, .halfAngle = 5.0f, .rayCount = 12},
      {.x = 50, .y = -10, .theta = -60, .halfAngle = 5.0f, .rayCount = 12},
      {.x = 25, .y = -30, .theta = -10, .halfAngle = 5.0f, .rayCount = 12},
  };
  const float SENSOR_MAX_RANGE = 355.0f;
  const uint8_t SENSOR_ALPHA = 128;
  const sf::Color SENSOR_COLOUR(255, 0, 255, SENSOR_ALPHA);

}  // namespace conf
