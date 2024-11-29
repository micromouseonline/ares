#pragma once
#include <SFML/Graphics.hpp>
#include <string>
namespace conf {
  // Window configuration
  const std::string AppName = "ARES Simulator for Micromouse";
  const sf::Vector2u WindowSize = {1900, 1050};
  const sf::Vector2f WindoSizef = static_cast<sf::Vector2f>(WindowSize);
  const sf::VideoMode VideoMode = {WindowSize.x, WindowSize.y, 32};
  const uint32_t FrameRate = 60;
  float const FrameDeltaTime = 1.0f / static_cast<float>(FrameRate);

}  // namespace conf
