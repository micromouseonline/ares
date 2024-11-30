#ifndef _OBJECT_H
#define _OBJECT_H

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include "collisions.h"
#include "robot-sensor.h"
#include "common/collisions.h"
/*

the RobotBody is a collection of shapes that can be rotated and moved. They are positioned at an
offset from the center of the robot.
The shapes are stored in a vector so that they can be used together to test for collisions.

TODO: perhaps this is the place to draw a sprite?
*/

class RobotBody {
 public:
  struct ShapeData {
    std::unique_ptr<sf::Shape> shape;
    sf::Vector2f originalOffset;
    sf::Vector2f rotatedOffset;
  };

  explicit RobotBody(const sf::Vector2f& center = {0, 0}) : m_center(center) { m_colour = sf::Color::White; }

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

  void configure_sensor_geometry(RobotBody& robot) {
    (void)robot;
    //    sensor_lfs.set_angle(robot.angle());
    //    sensor_lfs.set_half_angle((float)g_robot_state.sensor_half_angle);
    //    sensor_lds.set_angle(robot.angle());
    //    sensor_lds.set_half_angle((float)g_robot_state.sensor_half_angle);
    //    sensor_rds.set_angle(robot.angle());
    //    sensor_rds.set_half_angle((float)g_robot_state.sensor_half_angle);
    //    sensor_rfs.set_angle(robot.angle());
    //    sensor_rfs.set_half_angle((float)g_robot_state.sensor_half_angle);

    /// Update the sensor geometry. This is done after we have
    /// decided if we have collided or not so the angles and positions are correct
    /// It is all pretty cumbersome for now
    //    sf::Vector2f lfs_pos = rotatePoint(g_robot_state.lfs_offs, {0, 0}, robot.angle());
    //    sf::Vector2f lds_pos = rotatePoint(g_robot_state.lds_offs, {0, 0}, robot.angle());
    //    sf::Vector2f rds_pos = rotatePoint(g_robot_state.rds_offs, {0, 0}, robot.angle());
    //    sf::Vector2f rfs_pos = rotatePoint(g_robot_state.rfs_offs, {0, 0}, robot.angle());
    //
    //    sensor_lfs.set_origin(robot.position() + lfs_pos);
    //    sensor_lds.set_origin(robot.position() + lds_pos);
    //    sensor_rds.set_origin(robot.position() + rds_pos);
    //    sensor_rfs.set_origin(robot.position() + rfs_pos);
    //    float lfs_ang = -90 - g_robot_state.front_sensor_angle;
    //    float lds_ang = -180 + g_robot_state.side_sensor_angle;
    //    float rds_ang = 0 - g_robot_state.side_sensor_angle;
    //    float rfs_ang = -90 + g_robot_state.front_sensor_angle;
    //    sensor_lfs.set_angle(robot.angle() + lfs_ang);
    //    sensor_lds.set_angle(robot.angle() + lds_ang);
    //    sensor_rds.set_angle(robot.angle() + rds_ang);
    //    sensor_rfs.set_angle(robot.angle() + rfs_ang);
  }

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
  RobotSensor sensor_lfs;
  RobotSensor sensor_lds;
  RobotSensor sensor_rds;
  RobotSensor sensor_rfs;
};

#endif  // _OBJECT_H
