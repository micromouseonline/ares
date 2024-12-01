#pragma once
#include <SFML/Graphics.hpp>
#include <string>
#include "common/core.h"

namespace conf {

  // Maze Dimensions
  const float MazeSize = (16 * 180.0f) + 12.0f;  // 2892.0f;

  // Window configuration
  const std::string AppName = "ARES Simulator for Micromouse";
  const sf::Vector2u WindowSize = {1900, 1050};
  const sf::Vector2f WindoSizef = static_cast<sf::Vector2f>(WindowSize);
  const sf::VideoMode VideoMode = {WindowSize.x, WindowSize.y, 32};
  const sf::Color WindowBackGround = sf::Color(16, 16, 16, 255);

  const sf::View MazeView({MazeSize / 2.0f, MazeSize / 2.0f}, {MazeSize, MazeSize});
  const uint32_t FrameRate = 60;
  const float FrameDeltaTime = 1.0f / static_cast<float>(FrameRate);
  const float WindowPadding = 10.0f;
  const float MazeViewScreenSize = 964.0f;

  // maze colours
  const sf::Color KnownPresentColour = (sf::Color(255, 0, 0));
  const sf::Color knownAbsentColour = (sf::Color(0, 0, 0));
  const sf::Color UnknownColour = (sf::Color(64, 64, 64, 64));
  const sf::Color VirtualColour = (sf::Color(0, 255, 255));
  const sf::Color ErrorColour = (sf::Color(255, 0, 255, 128));
  const sf::Color MazeBaseColour = sf::Color(30, 30, 30, 64);

  // Robot configuration defaults
  // TODO: Add the body details
  enum WallSensorName { LFS = 0, LDS, RDS, RFS, SENSOR_COUNT };
  const SensorGeometry SensorDefaultOffsets[SENSOR_COUNT] = {
      {.x = 25, .y = -30, .theta = -10, .halfAngle = 5.0f, .rayCount = 32},
      {.x = 50, .y = -10, .theta = -60, .halfAngle = 5.0f, .rayCount = 32},
      {.x = 50, .y = +10, .theta = +60, .halfAngle = 5.0f, .rayCount = 32},
      {.x = 25, .y = 30, .theta = +10, .halfAngle = 5.0f, .rayCount = 32},
  };
  const float SENSOR_MAX_RANGE = 255.0f;
  const uint8_t SENSOR_ALPHA = 128;
  const sf::Color SENSOR_COLOUR(255, 0, 255, SENSOR_ALPHA);

}  // namespace conf
