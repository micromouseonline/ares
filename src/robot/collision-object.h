#ifndef _OBJECT_H
#define _OBJECT_H

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include "collisions.h"

class CollisionGeometry {
 public:
  struct ShapeData {
    std::unique_ptr<sf::Shape> shape;
    sf::Vector2f originalOffset;
    sf::Vector2f rotatedOffset;
  };

  explicit CollisionGeometry(const sf::Vector2f& center = {0, 0}) : m_center(center) { m_colour = sf::Color::White; }

  void addShape(std::unique_ptr<sf::Shape> shape, const sf::Vector2f& offset) {
    shape->setPosition(m_center + offset);
    shapedata.push_back({std::move(shape), offset, offset});
  }
  void set_colour(sf::Color colour) { m_colour = colour; }

  void rotate(float angle) {
    // Convert angle from degrees to radians
    m_angle += angle;
    while (m_angle > 360) {
      m_angle -= 360;
    }
    while (m_angle < 0) {
      m_angle += 360;
    }

    setRotation(m_angle);
  }

  void setPosition(float x, float y) { setPosition(sf::Vector2f(x, y)); }

  void setPosition(const sf::Vector2f& position) {
    m_center = position;
    for (const auto& item : shapedata) {
      item.shape->setPosition(m_center + item.rotatedOffset);
    }
  }

  void setRotation(float angle) {
    m_angle = angle;
    float radAngle = m_angle * (3.14159265359f / 180.f);
    float cosAngle = std::cos(radAngle);
    float sinAngle = std::sin(radAngle);

    for (auto& shape_data : shapedata) {
      auto& shape = shape_data.shape;
      auto& initialOffset = shape_data.originalOffset;
      auto& rotatedOffset = shape_data.rotatedOffset;
      rotatedOffset.x = initialOffset.x * cosAngle - initialOffset.y * sinAngle;
      rotatedOffset.y = initialOffset.x * sinAngle + initialOffset.y * cosAngle;
      shape->setPosition(m_center + rotatedOffset);
      shape->setRotation(angle);
    }
  }

  float angle() const { return m_angle; }

  sf::Vector2f position() const { return m_center; }

  void draw(sf::RenderWindow& window) {
    for (const auto& item : shapedata) {
      window.draw(*item.shape);
    }

    sf::CircleShape dot(8);
    dot.setOrigin(8, 8);
    dot.setFillColor(m_colour);
    //    dot.setFillColor(sf::Color::White);
    dot.setPosition(m_center);
    window.draw(dot);
  }

  bool collides_with(const sf::RectangleShape& rect) const {
    for (const auto& item : shapedata) {
      if (auto* circle = dynamic_cast<sf::CircleShape*>(item.shape.get())) {
        if (Collisions::circle_hits_aligned_rect(*circle, rect)) {  /// only axis aligned rectangles
          return true;
        }
      } else if (auto* this_rect = dynamic_cast<sf::RectangleShape*>(item.shape.get())) {
        if (Collisions::rectangles_overlap(rect, *this_rect)) {
          return true;
        }
      }
    }
    return false;
  }

 private:
  sf::Vector2f m_center;
  float m_angle = 0;
  sf::Color m_colour = sf::Color::White;
  std::vector<ShapeData> shapedata;  // List of shapes and their offsets
};

#endif  // _OBJECT_H
