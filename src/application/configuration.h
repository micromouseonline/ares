#pragma once
#include <SFML/Graphics.hpp>

namespace conf {
  // Window configuration
  sf::Vector2u const WindowSize = {1900, 1050};
  sf::Vector2f const WindoSizef = static_cast<sf::Vector2f>(WindowSize);
  sf::VideoMode const VideoMode = {WindowSize.x, WindowSize.y, 32};
  uint32_t const FrameRate = 60;
  float const FrameDeltaTime = 1.0f / static_cast<float>(FrameRate);

}
