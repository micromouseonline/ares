//
// Created by peter on 20/12/24.
//

#pragma once

#include <cmath>
#include "core.h"

class Pose {
 public:
  // Constructor to initialize the pose
  Pose(float x, float y, float theta) : m_x(x), m_y(y), m_omega(0), m_distance(0), m_theta(theta) {};

  // Default constructor
  Pose() : m_x(0), m_y(0), m_velocity(0), m_omega(0), m_distance(0), m_theta(0) {
  }

  // Getters for the pose components
  float getX() const {
    return m_x;
  }
  float getY() const {
    return m_y;
  }
  float getDistance() const {
    return m_distance;
  }
  float getTheta() const {
    return m_theta;
  };
  float getVelocity() const {
    return m_velocity;
  }
  float getOmega() const {
    return m_omega;
  }

  // Setters for the pose components
  void setX(float x) {
    m_x = x;
  }
  void setY(float y) {
    m_y = y;
  }
  void setVelocity(float v) {
    m_velocity = v;
  }
  void setOmega(float w) {
    m_omega = w;
  }
  void setDistance(float dist) {
    m_distance = dist;
  }
  void setTheta(float theta) {
    m_theta = theta;
  }

  /***
   * Pose is advanced by one time step of motion.
   * Simple Euler integration is used
   * Acceleration is not used
   * TODO: consider Verlet integration if errors
   *       accumulate over a long period. This is
   *       not likely though
   * @param delta_time - time step for the update
   */
  void advance(float delta_time) {
    float distance_change = m_velocity * delta_time;
    float angle_change = m_omega * delta_time;
    m_distance += distance_change;
    m_theta += angle_change;
    m_theta = std::fmod(m_theta + 360.0f, 360.0f);
    m_x += distance_change * std::cos(m_theta * RADIANS);
    m_y += distance_change * std::sin(m_theta * RADIANS);
  }

 private:
  float m_x;
  float m_y;
  float m_velocity;
  float m_omega;
  float m_distance;
  float m_theta;
};
