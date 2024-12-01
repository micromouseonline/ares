#ifndef OBJECT_H
#define OBJECT_H

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include "common/collisions.h"
#include "drawing.h"
#include "robot-wall-sensor.h"
#include "robot/robot.h"
/**
 * The RobotBody holds the physical arrangement of the robot and describes its appearance on
 * the display.
 *
 * There is a list of shapes that is used for collision detection. These shapes should be simple
 * circles and rectangles with dimensions in mm. On screen, 1mm = 1 pixel.
 *
 * There can also be a sprite associated with the body. If present, it will be drawn last, over
 * the shapes used for collisions.
 *
 * Also stored here are the relative locations of the sensors.
 *
 * Everything about the appearance of the robot and its sensors is rendered to the screen with
 * the draw method.
 *
 */

class RobotBody {
 public:
  struct ShapeData {
    std::unique_ptr<sf::Shape> shape;
    sf::Vector2f originalOffset;
    sf::Vector2f rotatedOffset;
  };

  RobotBody() {
    createBody();
    m_sensors[conf::LFS].SetGeometry(conf::SensorDefaultOffsets[conf::LFS]);
    m_sensors[conf::LDS].SetGeometry(conf::SensorDefaultOffsets[conf::LDS]);
    m_sensors[conf::RDS].SetGeometry(conf::SensorDefaultOffsets[conf::RDS]);
    m_sensors[conf::RFS].SetGeometry(conf::SensorDefaultOffsets[conf::RFS]);

    // Initialize static obstacles outside draw
    block.setSize({720, 90});
    block.setPosition(800, 1090);
    block.setFillColor(sf::Color::Cyan);

    block2.setSize({90, 90});
    block2.setPosition(90 * 12, 90 * 17);
    block2.setFillColor(sf::Color::Cyan);
  }

  void createBody() {
    auto head = std::make_unique<sf::CircleShape>(38);
    head->setOrigin(38, 38);
    head->setFillColor(sf::Color(0, 66, 0, 255));
    addShape(std::move(head), sf::Vector2f(22, 0));

    auto body = std::make_unique<sf::RectangleShape>(sf::Vector2f(60, 76));
    body->setFillColor(sf::Color(90, 76, 0, 255));
    body->setOrigin(38, 38);
    addShape(std::move(body), sf::Vector2f(0, 0));

    auto dot = std::make_unique<sf::CircleShape>(8);
    dot->setOrigin(8, 8);
    dot->setFillColor(m_colour);
    addShape(std::move(dot), sf::Vector2f(0, 0));
  }

  void setRobot(Robot& robot) { m_robot = &robot; }

  void addShape(std::unique_ptr<sf::Shape> shape, const sf::Vector2f& offset) {
    shape->setPosition(m_center + offset);
    bodyShapes.push_back({std::move(shape), offset, offset});
  }

  void set_colour(sf::Color colour) { m_colour = colour; }

  void setPosition(float x, float y) { setPosition(sf::Vector2f(x, y)); }

  void setPosition(const sf::Vector2f& position) {
    m_center = position;
    for (const auto& item : bodyShapes) {
      item.shape->setPosition(m_center + item.rotatedOffset);
    }
  }

  void setRotation(float angle) {
    m_angle = angle;
    float radAngle = m_angle * (3.14159265359f / 180.f);
    float cosAngle = std::cos(radAngle);
    float sinAngle = std::sin(radAngle);

    for (auto& shape_data : bodyShapes) {
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

  void updateSensorGeometry() {
    float angle = 0;
    sf::Vector2f pos{0, 0};
    if (m_robot) {
      /// angle is negative because the screen y-axis is inverted
      pos = m_robot->getPose();
      angle = -(m_robot->getOrientation());
    }

    for (auto& sensor : m_sensors) {
      sensor.set_origin(pos + rotatePoint({sensor.getGeometry().x, sensor.getGeometry().y}, {0, 0}, angle));
      sensor.set_angle(angle + sensor.getGeometry().theta);
    }
  }

  void draw(sf::RenderWindow& window) {
    float angle = 0;
    sf::Vector2f pos{90, 90};
    if (m_robot) {
      pos = m_robot->getPose();
      angle = -(m_robot->getOrientation());
    }

    setPosition(pos);
    setRotation(angle);
    updateSensorGeometry();

    for (const auto& item : bodyShapes) {
      window.draw(*item.shape);
    }

    // Draw static obstacles
    /// TODO: these will come from the maze
    window.draw(block);
    window.draw(block2);
    std::vector<sf::RectangleShape> obstacles{block, block2};

    // Update and draw sensors
    for (auto& sensor : m_sensors) {
      sensor.update(obstacles);
      sensor.draw(window);
    }

    // Draw the direction arrow
    Vec2 pointer = Vec2::fromDegrees(angle) * 100;
    Drawing::draw_vector_arrow(window, m_center, sf::Vector2f(pointer), 15.0);
  }

  bool collides_with(const sf::RectangleShape& rect) const {
    for (const auto& item : bodyShapes) {
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

  const RobotWallSensor& getSensor(int i) {
    if (i >= conf::SENSOR_COUNT) {
      throw std::out_of_range("Sensor index out of range");
    }
    return m_sensors[i];  //
  }

 private:
  RobotWallSensor m_sensors[conf::SENSOR_COUNT];
  sf::Vector2f m_center;
  float m_angle = 0;
  sf::Color m_colour = sf::Color::White;
  std::vector<ShapeData> bodyShapes;  // List of shapes and their offsets
  Robot* m_robot = nullptr;
  sf::RectangleShape block;
  sf::RectangleShape block2;
};

#endif  // OBJECT_H
