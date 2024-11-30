//
// Created by peter on 04/11/24.
//

#ifndef IMGUI_SFML_STARTER_UTILS_H
#define IMGUI_SFML_STARTER_UTILS_H

#include <SFML/Graphics.hpp>
#include <cmath>

inline float exponential_filter(float var, float new_value, float alpha = 0.5) {
  var = alpha * var + (1 - alpha) * new_value;
  return var;
}

// Returns distance between points
inline float distance(const sf::Vector2f& a, const sf::Vector2f& b) {
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline bool isWithinBounds(const sf::Vector2u& size, int x, int y) {
  return x >= 0 && x < static_cast<int>(size.x) && y >= 0 && y < static_cast<int>(size.y);
}

inline sf::Color getColorAtPixel(const sf::Image& image, int x, int y) {
  return isWithinBounds(image.getSize(), x, y) ? image.getPixel(x, y) : sf::Color::Transparent;
}

inline sf::Color getColorAtPixel(const sf::Image& image, sf::Vector2f point) {
  int x = (int)point.x;
  int y = (int)point.y;
  return isWithinBounds(image.getSize(), x, y) ? image.getPixel(x, y) : sf::Color::Transparent;
}

inline sf::Vector2f rotatePoint(const sf::Vector2f& point, const sf::Vector2f& center, float angle) {
  // Convert angle from degrees to radians
  float radian = angle * (3.14159265 / 180.0);

  // Calculate the new x and y positions using the rotation matrix
  float newX = std::cos(radian) * (point.x - center.x) - std::sin(radian) * (point.y - center.y) + center.x;
  float newY = std::sin(radian) * (point.x - center.x) + std::cos(radian) * (point.y - center.y) + center.y;

  return sf::Vector2f(newX, newY);
}
#endif  // IMGUI_SFML_STARTER_UTILS_H
