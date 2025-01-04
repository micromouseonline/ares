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

#ifndef CONTROLLER_TRAPEZOID_H_
#define CONTROLLER_TRAPEZOID_H_

#include <cstddef>
#include "trajectory.h"

/***
 * I found the original code for this trapezoidal profiler as part of the
 * repository for MIZUHO, a micromouse by user 12312.
 *
 * https://github.com/idt12312/MIZUHO
 *
 * It has been refactored and edited to match more closely with my derivation
 * of the equations which can be found in the docs directory
 */
class Trapezoid : public Trajectory {
 public:
  // Default constructor initializes the trapezoid profile to zero distance and velocities
  Trapezoid()
      : m_s(0),        // Total distance to travel
        m_v1(0),       // Starting velocity
        m_v_limit(0),  // Maximum velocity
        m_v2(0),       // Ending velocity
        m_a(0),        // Acceleration value
        m_dir(0)       // Direction of motion (+1 or -1)
  {
  }

  // Parameterized constructor initializes the trapezoid motion profile
  Trapezoid(float dist, float v_start, float v_max, float v_end, float accel, float dt = 0.001f)
      : m_s(std::abs(dist)),         // Absolute distance to ensure positive magnitude
        m_v1(std::abs(v_start)),     // Ensure positive velocity
        m_v_limit(std::abs(v_max)),  // Ensure positive max velocity
        m_v2(std::abs(v_end)),       // Ensure positive end velocity
        m_a(std::abs(accel)),        // Ensure positive acceleration
        m_dir(sign(dist))            // Direction of motion (+1 or -1)
  {
    setDeltaTime(dt);
  }

  ~Trapezoid() override = default;

  // Initialize the motion profile
  // TODO what if Vmax < V1 or V2?
  void init(const Pose &p) override {
    Trajectory::init(p);
    // Calculate the steps required for acceleration and deceleration phases (p1 and p3)
    m_p1 = static_cast<int>(-(2 * m_v1 - std::sqrt(2 * m_v1 * m_v1 + 2 * m_v2 * m_v2 + 4 * m_s * m_a)) / (2 * m_a * m_delta_time));
    m_p3 = static_cast<int>(-(2 * m_v2 - std::sqrt(2 * m_v1 * m_v1 + 2 * m_v2 * m_v2 + 4 * m_s * m_a)) / (2 * m_a * m_delta_time));

    // Recalculate acceleration to ensure distance matches the provided input
    m_a = (m_s - m_delta_time * ((float)m_p1 * m_v1 + (float)m_p3 * m_v2)) /  //
          (0.5f * m_delta_time * m_delta_time * (float)(m_p1 * m_p1 + m_p3 * m_p3));

    // Check if maximum velocity is reached during acceleration
    if (m_v1 + m_a * (float)m_p1 * m_delta_time < m_v_limit) {
      m_p2 = 0;  // No constant velocity phase
    } else {
      // Adjust acceleration and deceleration phases to reach maximum velocity
      m_p1 = static_cast<int>((m_v_limit - m_v1) / (m_delta_time * m_a));
      m_p3 = static_cast<int>((m_v_limit - m_v2) / (m_delta_time * m_a));

      float t1 = (float)m_p1 * m_delta_time;
      float t3 = (float)m_p3 * m_delta_time;
      // Calculate the remaining distance for the constant velocity phase
      float cruise_dist = m_s - (0.5f * m_a * (t1 * t1 + t3 * t3) + m_v1 * t1 + m_v2 * t3);

      // Calculate the number of steps at maximum velocity
      m_p2 = static_cast<int>(cruise_dist / (m_delta_time * m_v_limit));

      // Adjust the actual maximum velocity for exact result
      m_v_limit = cruise_dist / (m_delta_time * (float)m_p2);
    }

    // Reset step count and set motion as active
    reset();
  }

  // Calculate the next velocity step in the profile
  float next() override {
    float v = 0.0f;
    if (m_current_step <= m_p1) {  // Phase 1: Acceleration
      v = m_v1 + m_a * m_delta_time * (float)m_current_step;
    } else if (m_current_step <= m_p1 + m_p2) {  // Phase 2: Constant velocity
      v = m_v_limit;
    } else if (m_current_step < m_p1 + m_p2 + m_p3) {  // Phase 3: Deceleration
      v = m_v2 + m_a * m_delta_time * float(m_p1 + m_p2 + m_p3 - m_current_step);
    }

    // Increment step count if motion is not finished
    if (!m_finished) {
      m_current_step++;
    }

    // Check if motion is finished
    if (m_current_step >= m_p1 + m_p2 + m_p3) {
      m_finished = true;
      v = m_v2;  // Final velocity
    }
    m_current_pose.setVelocity(v);
    m_current_pose.advance(m_delta_time);
    return m_dir * v;  // Apply direction to velocity
  }

 private:
  // Member variables
  float m_s;        // Total distance to travel
  float m_v1;       // Starting velocity
  float m_v_limit;  // Maximum velocity
  float m_v2;       // Ending velocity
  float m_a;        // Acceleration value

  float m_dir;  // Direction of motion (+1 or -1)
  int m_p1;     // Steps in acceleration phase
  int m_p2;     // Steps in constant velocity phase
  int m_p3;     // Steps in deceleration phase
};

#endif /* CONTROLLER_TRAPEZOID_H_ */
