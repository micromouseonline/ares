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
    sensor_lfs.set_angle(0);
    sensor_lfs.set_half_angle(5.0);
    sensor_lfs.set_ray_count(32);
    m_robot_sensors.push_back(sensor_lfs);
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
    std::lock_guard<std::mutex> lock(m_BodyMutex);
    for (auto& sensor : m_robot_sensors) {
      sensor.set_angle(m_Robot->GetOrientation());
      sensor.set_origin(m_Robot->GetPose());
    }
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
    float angle = 0;
    sf::Vector2f pos{90, 90};
    if (m_Robot) {
      pos = m_Robot->GetPose();
      angle = m_Robot->GetOrientation() + 90;
    }
    setPosition(pos);
    setRotation(angle);

    for (const auto& item : bodyShapes) {
      window.draw(*item.shape);
    }
    sf::CircleShape dot(8);
    dot.setOrigin(8, 8);
    dot.setFillColor(m_colour);
    dot.setPosition(m_center);
    window.draw(dot);
    sf::RectangleShape block({300, 300});
    block.setPosition(1000, 900);
    block.setFillColor(sf::Color::Cyan);
    window.draw(block);

    std::vector<sf::RectangleShape> obstacles;
    obstacles.clear();
    obstacles.push_back(block);
    for (auto& sensor : m_robot_sensors) {
      sensor.set_angle(angle - 90);
      sensor.set_origin(pos);
      sensor.draw(window);
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

 private:
  sf::Vector2f m_center;
  float m_angle = 0;
  sf::Color m_colour = sf::Color::White;
  std::vector<ShapeData> bodyShapes;  // List of shapes and their offsets
  std::vector<RobotWallSensor> m_robot_sensors;
  RobotWallSensor sensor_lfs;
  RobotWallSensor sensor_lds;
  RobotWallSensor sensor_rds;
  RobotWallSensor sensor_rfs;
  Robot* m_Robot = nullptr;
  std::mutex m_BodyMutex;
};

#endif  // _OBJECT_H
