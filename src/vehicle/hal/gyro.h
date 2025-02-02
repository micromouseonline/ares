//
// Created by peter on 02/02/25.
//

#pragma once
#include "common/core.h"

/***
 * Stub for an IMU. Only the gyro is used.
 *
 * Normally only used internally from the Vehicle, it is made available
 * for debugging
 *
 * TODO: Hide the IMU from the users of Vehicle
 */

class Gyro {
  float omega() {
    return 0.0f;
  }

  float theta() {
    return 0.0f;
  }

  void update(float dt) {
    float angle = m_theta + m_omega * dt;
    m_theta = normalizeAngle(angle);
  }

 private:
  float m_omega = 0.0f;
  float m_theta = 0.0f;
};
