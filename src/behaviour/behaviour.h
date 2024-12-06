//
// Created by peter on 22/11/24.
//

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

/**
 *
 * The RobotControl class is responsible for the behavior of the robot in the
 * environment made available by the RobotDisplay class.
 *
 * For a micromouse, this includes path planning, searching, mapping,
 * and other decision-making behaviors. It can be adapted for other types of robots.
 *
 * The RobotControl class holds an instance of the Robot class and a logical
 * representation of the Maze. The physical maze is maintained and updated by
 * the main thread.
 *
 * Synchronization between RobotControl and RobotDisplay is managed using
 * condition variables. RobotControl sets a request flag and calls notify_all().
 * RobotDisplay responds by calculating sensor readings appropriate to the current
 * robot state and updating the Robot instance.
 *
 * Core Features of RobotControl
 * Behavioral Logic:
 *    - Implements decision-making algorithms such as path planning, mapping, and navigation.
 *    - Reacts to sensor data (e.g., front distance, wall proximity) for obstacle avoidance or exploration.
 *
 * Synchronization:
 *    - Uses a std::condition_variable to coordinate updates between threads:
 *    - Notifies the RobotDisplay thread to update sensor data.
 *    - Waits for the updated data to proceed with control logic.
 *
 * Robot Interaction:
 *    - Directly interacts with the Robot instance to access pose, velocity, or other state data.
 *
 * Environment Access:
 *    - Maintains a reference to a Maze object, which represents the logical environment
 *      for pathfinding and navigation.
 *
 * mRobot, mMaze and mSensorData are private members of RobotControl accessed
 * through methods that ensure thread safety. RobotControl manages all synchronization
 * between threads.
 *
 */

#include <SFML/Graphics.hpp>
#include "robot/sensor-data.h"

class Behaviour {
 public:
  Behaviour() : m_velocity(0.0f), m_omega(0.0f), m_center(0.0f, 0.0f), m_radius(0.0f) {};

  // Updates the control logic with the current sensor data
  void update() {};

  // Get the current velocities (linear and angular)
  float getLinearVelocity() const {
    return m_velocity;  //
  };

  float getAngularVelocity() const {
    return m_omega;  //
  };

  // Set a circular trajectory for the robot
  void setCircularTrajectory(const sf::Vector2f& center, float radius, float angularSpeedDegrees) {
    m_center = center;
    m_radius = radius;
    m_omega = angularSpeedDegrees;  // Store angular speed in degrees per second
    m_velocity = radius * (m_omega * 3.14159f / 180.0f);
    // Convert angular speed to radians for calculation
  }

 private:
  // Current control outputs
  float m_velocity;       // Forward velocity (units per second)
  float m_omega;          // Angular velocity (radians per second)
  sf::Vector2f m_center;  // Center of the circular trajectory
  float m_radius;         // Radius of the circular trajectory
};

#endif  // ROBOT_CONTROL_H
