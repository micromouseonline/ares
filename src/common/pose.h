//
// Created by peter on 20/12/24.
//

#ifndef MOUSE_SIMULATOR_POSE_H
#define MOUSE_SIMULATOR_POSE_H

#include "common/vec2.h"

class Pose {
 public:
  // Constructor to initialize the pose
  Pose(float x, float y, float theta) : m_position(x, y), m_velocity(), m_distance(0, theta) {};

  // Default constructor
  Pose() : m_position(), m_velocity(), m_distance() {
  }

  // Getters for the pose components
  float getX() const {
    return m_position.x;
  }
  float getY() const {
    return m_position.y;
  }
  float getDistance() const {
    return m_distance.x;
  }
  float getTheta() const {
    return m_distance.y;
  };
  float getVelocity() const {
    return m_velocity.x;
  }
  float getOmega() const {
    return m_velocity.y;
  }

  // Setters for the pose components
  void setX(float x) {
    m_position.x = x;
  }
  void setY(float y) {
    m_position.y = y;
  }
  void setVelocity(float v) {
    m_velocity.x = v;
  }
  void setOmega(float w) {
    m_velocity.y = w;
  }
  void setDistance(float dist) {
    m_distance.x = dist;
  }
  void setTheta(float theta) {
    m_distance.y = theta;
  }

 private:
  Vec2 m_position;  // x,y world coordinates
  Vec2 m_velocity;  // linear and angular velocities
  Vec2 m_distance;  // linear and angular distance
};

#endif  // MOUSE_SIMULATOR_POSE_H
