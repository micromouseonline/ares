/******************************************************************************
 * Project: mazerunner32-ares                                                 *
 * -----                                                                      *
 * Copyright 2022 - 2024 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef BEHAVIOUR_CUBIC_H
#define BEHAVIOUR_CUBIC_H
/***
 * the Cubic trajectory is used exclusively for smooth turns.
 *
 * The technique is easy to calculate and gives exact values
 * for angular velocity at any point along the trajectory.
 *
 * A major benefit is the ability to run the same trajectory
 * at any speed (ignoring slip of course).
 *
 * It is guaranteed to finish with omega = 0 and can be made
 * more tight or loose just by adjusting the value of the length.
 *
 */

#include <cmath>
#include "behaviour/trajectory.h"
#include "common/pose.h"
#include "cubic_parameters.h"

class Cubic : public Trajectory {
 public:
  Cubic()
      : m_length(0),
        m_angle(0),
        m_velocity(0) {
    m_type = CUBIC;
  }

  Cubic(float length, float angle, float velocity)
      :  //
        m_length(length),
        m_angle(angle),
        m_velocity(velocity) {
    m_type = CUBIC;
  }

  virtual ~Cubic() = default;

  // Initialize the motion profile
  void init(const Pose& pose) override {
    m_start_pose = pose;
    m_cubic_dist = m_length;
    m_distance = 0;
    m_cubic_constant = 6.0f * m_angle * RADIANS / (m_cubic_dist * m_cubic_dist * m_cubic_dist);
    reset();
  };

  // set the control variables to the start of the trajectory, ready for stepping through
  virtual void begin() override {
    m_current_step = 0;
    m_distance = 0;
    m_finished = false;
  };

  // Calculate the next velocity step in the profile
  virtual void update() override {
    m_distance = m_current_step * m_delta_time * m_velocity;
    float remaining = m_cubic_dist - m_distance;
    float t = m_distance * remaining;
    float omega = m_velocity * m_cubic_constant * t * DEGREES;
    if (remaining <= 0) {
      m_current_pose.setOmega(0);
      m_finished = true;
    } else {
      m_current_pose.advance(m_velocity, omega, m_delta_time);
      m_current_step++;
    }
  }

 private:
  float m_length = 0;
  float m_angle = 0;
  float m_velocity = 0;
  float m_cubic_dist = 0;
  float m_cubic_constant = 0;
  float m_distance = 0;
};
inline Cubic cubicTraj(0, 0, 0);  // Placeholder for Cubic

#endif /* BEHAVIOUR_CUBIC_H */
