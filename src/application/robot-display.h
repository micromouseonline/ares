//
// Created by peter on 02/11/2024.
//

#ifndef ROBOTVIEW_H
#define ROBOTVIEW_H

#include <SFML/Graphics.hpp>
#include <iostream>
#include <mutex>
#include <optional>
#include "application/drawing.h"
#include "application/event_observer.h"
#include "robot/robot.h"
#include "utils/thread-safe-queue.h"

/***
 * The RobotDisplay class renders the robot to the screen and handles any
 * user interaction - for instance, starting, stopping, stepping,
 * mode changes...
 *
 * Instances of this class will run in the main application thread and so it
 * must be made thread safe.
 *
 * To communicate with the RobotControl class, shared variables must be used
 * and guarded with a mutex. Condition variables can also be used to signal
 * changes in state.
 *
 * Thread safe queues can also be used to pass messages between threads. This
 * is particularly useful for simulating logging and telemetry.
 *
 * Core Features of RobotDisplay
 * Rendering:
 *    - Visualize the robot's position and orientation in the simulation environment.
 *    - Render additional elements, such as the maze or sensor data, for better debugging and feedback.
 * User Interaction:
 *    - Handle events such as starting, stopping, stepping the simulation, or changing modes.
 * Thread Safety:
 *    - Use proper synchronization (std::mutex and std::condition_variable) when interacting with
 *    RobotControl to avoid race conditions.
 * Simulation Control:
 *    - Provide an interface for controlling the simulation (e.g., play/pause, reset, or single-step mode).
 *
 * The RobotDisplay runs in the main application thread.
 *
 */

class RobotDisplay : IEventObserver {
 public:
  static void Draw(sf::RenderWindow& window, const sf::Vector2f& pose, float orientation) {
    // Draw the robot as a circle
    sf::CircleShape robotShape(ROBOT_RADIUS);
    robotShape.setOrigin(ROBOT_RADIUS, ROBOT_RADIUS);  // Center the circle at its position
    robotShape.setPosition(pose);
    robotShape.setFillColor(sf::Color::Blue);  // Robot body color

    // Draw the robot's heading as a line
    orientation *= RADIANS;

    // Calculate the heading endpoint
    sf::Vector2f headingVector{HEADING_LINE_LENGTH * std::cos(orientation), HEADING_LINE_LENGTH * std::sin(orientation)};

    // Render the robot
    window.draw(robotShape);

    // Render the heading
    draw_vector_arrow(window, pose, headingVector, 25.0, sf::Color::Red);
  };

 private:
  static constexpr float ROBOT_RADIUS = 35.0f;         // Radius of the robot visualization
  static constexpr float HEADING_LINE_LENGTH = 50.0f;  // Length of the heading line

 public:
};

////////////////////////////////////////////////////////////////////////////////////////
#if 0

Stuff from the current visual representation

class RobotDisplayView {
 public:
  RobotDisplayView(Robot& robot) : m_robot(robot) {
    m_shape.setSize(sf::Vector2f(robot.m_width, robot.m_height));
    m_shape.setOrigin(robot.m_width / 2, robot.m_height / 2);
    m_shape.setFillColor(sf::Color::Green);
    m_shape.setPosition(m_robot.m_x, m_robot.m_y);
    sf::Image image;
    image.create(robot.m_width, robot.m_height, sf::Color::Blue);
    m_texture.loadFromImage(image);
    m_sprite.setTexture(m_texture);
    m_sprite.setOrigin(robot.m_origin_x, robot.m_origin_y);
    m_sprite.setPosition(m_robot.m_x, m_robot.m_y);
  }

  bool load_texture(const std::string& filename) {
    if (!m_texture.loadFromFile(filename)) {
      std::cerr << "Unable to load texture\n";
      return false;
    }
    m_sprite.setTexture(m_texture);
    m_sprite.setTextureRect(sf::IntRect(0, 0, m_robot.m_width, m_robot.m_height));
    m_sprite.setOrigin(m_robot.m_origin_x, m_robot.m_origin_y);
    update();
    return true;
  }

  void update() {
    m_shape.rotate(m_robot.m_angle);
    m_shape.setPosition(m_robot.m_x, m_robot.m_y);
    m_sprite.setRotation(m_robot.m_angle);
    m_sprite.setPosition(m_robot.m_x, m_robot.m_y);
    // setTextureRect(sf::IntRect(m_robot.m_state * m_robot.m_width, 0, m_robot.m_width, m_robot.m_height));
    //    setRotation(m_robot.m_angle);
    //    setPosition(m_robot.m_x, m_robot.m_y);
  }
  void draw(sf::RenderTarget& target) {
    if (m_sprite.getTexture()) {
      target.draw(m_sprite);
    } else {
      target.draw(m_shape);
    }
  }

 private:
  Robot& m_robot;
  sf::RectangleShape m_shape;
  sf::Sprite m_sprite;
  sf::Texture m_texture;
};
#endif
#endif  // ROBOTVIEW_H
