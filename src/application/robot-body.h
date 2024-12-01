#ifndef _OBJECT_H
#define _OBJECT_H

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include "common/collisions.h"
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
    auto head = std::make_unique<sf::CircleShape>(38);
    head->setOrigin(38, 38);
    head->setFillColor(sf::Color(0, 66, 0, 255));
    addShape(std::move(head), sf::Vector2f(0, -31));
    auto body = std::make_unique<sf::RectangleShape>(sf::Vector2f(76, 62));
    body->setFillColor(sf::Color(0, 76, 0, 255));
    body->setOrigin(38, 31);
    addShape(std::move(body), sf::Vector2f(0, 0));
    sensor_lfs.SetGeometry(conf::SensorDefaultOffsets[0]);
    sensor_lds.SetGeometry(conf::SensorDefaultOffsets[1]);
    sensor_rds.SetGeometry(conf::SensorDefaultOffsets[2]);
    sensor_rfs.SetGeometry(conf::SensorDefaultOffsets[3]);

    m_robot_sensors.push_back(sensor_lfs);
    m_robot_sensors.push_back(sensor_lds);
    m_robot_sensors.push_back(sensor_rds);
    m_robot_sensors.push_back(sensor_rfs);
  };

  void setRobot(Robot& robot) { m_Robot = &robot; }

  void addShape(std::unique_ptr<sf::Shape> shape, const sf::Vector2f& offset) {
    shape->setPosition(m_center + offset);
    bodyShapes.push_back({std::move(shape), offset, offset});
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

  void updateSensorGeometry(Robot& robot) {
    //    (void)robot;
    float angle = 0;
    sf::Vector2f pos{90, 90};
    if (m_Robot) {
      pos = m_Robot->GetPose();
      angle = m_Robot->GetOrientation() + 90;
    }
    sensor_lfs.set_angle(angle + sensor_lfs.getGeometry().theta - 90);
    sensor_lds.set_angle(angle - sensor_lds.getGeometry().theta - 180);
    sensor_rds.set_angle(angle - sensor_rds.getGeometry().theta - 0);
    sensor_rfs.set_angle(angle + sensor_rfs.getGeometry().theta - 90);

    /// Update the sensor geometry. This is done after we have
    /// decided if we have collided or not so the angles and positions are correct
    /// It is all pretty cumbersome for now
    sf::Vector2f lfs_pos = rotatePoint({sensor_lfs.getGeometry().x, sensor_lfs.getGeometry().y}, {0, 0}, angle);
    sf::Vector2f lds_pos = rotatePoint({sensor_lds.getGeometry().x, sensor_lds.getGeometry().y}, {0, 0}, angle);
    sf::Vector2f rds_pos = rotatePoint({sensor_rds.getGeometry().x, sensor_rds.getGeometry().y}, {0, 0}, angle);
    sf::Vector2f rfs_pos = rotatePoint({sensor_rfs.getGeometry().x, sensor_rfs.getGeometry().y}, {0, 0}, angle);

    sensor_lfs.set_origin(pos + lfs_pos);
    sensor_lds.set_origin(pos + lds_pos);
    sensor_rds.set_origin(pos + rds_pos);
    sensor_rfs.set_origin(pos + rfs_pos);
  }

  void draw(sf::RenderWindow& window) {
    float angle = 0;
    sf::Vector2f pos{90, 90};
    if (m_Robot) {
      pos = m_Robot->GetPose();
      angle = m_Robot->GetOrientation() + 90;
    }
    setPosition(pos);
    setRotation(angle);
    updateSensorGeometry(*m_Robot);
    for (const auto& item : bodyShapes) {
      window.draw(*item.shape);
    }
    sf::CircleShape dot(8);
    dot.setOrigin(8, 8);
    dot.setFillColor(m_colour);
    dot.setPosition(m_center);
    window.draw(dot);

    sf::RectangleShape block({720, 90});
    block.setPosition(800, 1090);
    block.setFillColor(sf::Color::Cyan);
    window.draw(block);

    sf::RectangleShape block2({90, 90});
    block2.setPosition(90 * 12, 90 * 17);
    block2.setFillColor(sf::Color::Cyan);
    window.draw(block2);

    std::vector<sf::RectangleShape> obstacles;
    obstacles.clear();
    obstacles.push_back(block);
    obstacles.push_back(block2);
    sensor_lfs.update(obstacles);
    sensor_lds.update(obstacles);
    sensor_rds.update(obstacles);
    sensor_rfs.update(obstacles);

    sensor_lfs.draw(window);
    sensor_lds.draw(window);
    sensor_rds.draw(window);
    sensor_rfs.draw(window);
    for (auto& sensor : m_robot_sensors) {
      //      sensor.set_angle(angle - 90);
      sensor.set_origin(pos);
      //      sensor.draw(window);
      sensor.update(obstacles);
    }
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
  RobotWallSensor sensor_lfs;
  RobotWallSensor sensor_lds;
  RobotWallSensor sensor_rds;
  RobotWallSensor sensor_rfs;

 private:
  sf::Vector2f m_center;
  float m_angle = 0;
  sf::Color m_colour = sf::Color::White;
  std::vector<ShapeData> bodyShapes;  // List of shapes and their offsets
  std::vector<RobotWallSensor> m_robot_sensors;
  Robot* m_Robot = nullptr;
  std::mutex m_BodyMutex;
};

#endif  // _OBJECT_H
