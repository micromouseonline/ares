/*
 * Trapezoid.h
 *
 *  Created on: 2016/11/04
 *      Author: idt12312
 */

#ifndef CONTROLLER_TRAPEZOID_H_
#define CONTROLLER_TRAPEZOID_H_

#include <cstddef>

/***
 * this is the trapezoidal calculation used in MIZUMO, re-written for a little more clarity
 * Not a lot more clarity though
 */
class Trapezoid {
 public:
  // Default constructor initializes the trapezoid profile to zero distance and velocities
  Trapezoid()
      : m_distance(0),        // Total distance to travel
        m_dir(0),             // Direction of motion (+1 or -1)
        m_v_start(0),         // Starting velocity
        m_v_max(0),           // Maximum velocity
        m_v_end(0),           // Ending velocity
        m_accel(0),           // Acceleration value
        m_deltaTime(0.001f),  // Time step size
        m_step_count(0),      // Step counter for velocity progression
        m_finished(true)      // Motion state flag
  {
  }

  // Parameterized constructor initializes the trapezoid motion profile
  Trapezoid(float dist, float v_start, float v_max, float v_end, float accel, float dt = 0.001f)
      : m_distance(std::abs(dist)),    // Absolute distance to ensure positive magnitude
        m_dir(sign(dist)),             // Direction of motion (+1 or -1)
        m_v_start(std::abs(v_start)),  // Ensure positive velocity
        m_v_max(std::abs(v_max)),      // Ensure positive max velocity
        m_v_end(std::abs(v_end)),      // Ensure positive end velocity
        m_accel(std::abs(accel)),      // Ensure positive acceleration
        m_deltaTime(dt),               // Time step size
        m_step_count(0),               // Initialize step count
        m_finished(true)               // Start with motion as finished
  {
  }

  virtual ~Trapezoid() = default;

  // Initialize the motion profile
  void begin() {
    float v1 = m_v_start;
    float v2 = m_v_max;
    float v3 = m_v_end;
    float s = m_distance;
    float a = m_accel;
    float dt = m_deltaTime;
    float s1 = 0;
    float s2 = 0;
    float s3 = 0;

    // Calculate the steps required for acceleration and deceleration phases (p1 and p3)
    m_p1 = static_cast<int>(-(2 * v1 - std::sqrt(2 * v1 * v1 + 2 * v3 * v3 + 4 * s * a)) / (2 * a * dt));
    m_p3 = static_cast<int>(-(2 * v3 - std::sqrt(2 * v1 * v1 + 2 * v3 * v3 + 4 * s * a)) / (2 * a * dt));

    // Recalculate acceleration to ensure distance matches the provided input
    a = (s - dt * ((float)m_p1 * v1 + (float)m_p3 * v3)) /  //
        (0.5f * dt * dt * (float)(m_p1 * m_p1 + m_p3 * m_p3));

    // Check if maximum velocity is reached during acceleration
    if (v1 + a * (float)m_p1 * dt < v2) {
      m_p2 = 0;  // No constant velocity phase
    } else {
      // Adjust acceleration and deceleration phases to reach maximum velocity
      m_p1 = static_cast<int>((v2 - v1) / (dt * a));
      m_p3 = static_cast<int>((v2 - v3) / (dt * a));

      float t1 = (float)m_p1 * dt;
      float t3 = (float)m_p3 * dt;
      // Calculate the remaining distance for the constant velocity phase
      float remaining_distance = s - (0.5f * a * ((t1) * (t1) + (t3) * (t3)) + v1 * t1 + v3 * t3);

      // Calculate the number of steps at maximum velocity
      m_p2 = static_cast<int>(remaining_distance / (dt * v2));
      float t2 = (float)m_p2 * dt;

      // Adjust the actual maximum velocity to ensure distance consistency
      v2 = remaining_distance / (dt * (float)m_p2);
      m_v_max = v2;
      m_accel = a;
    }

    // Reset step count and set motion as active
    m_step_count = 0;
    m_finished = false;
  }

  // Calculate the next velocity step in the profile
  float next() {
    float v = 0.0f;
    if (m_step_count <= m_p1) {  // Phase 1: Acceleration
      v = m_v_start + m_accel * m_deltaTime * (float)m_step_count;
    } else if (m_step_count <= m_p1 + m_p2) {  // Phase 2: Constant velocity
      v = m_v_max;
    } else if (m_step_count < m_p1 + m_p2 + m_p3) {  // Phase 3: Deceleration
      v = m_v_end + m_accel * m_deltaTime * float(m_p1 + m_p2 + m_p3 - m_step_count);
    }

    // Increment step count if motion is not finished
    if (!m_finished) {
      m_step_count++;
    }

    // Check if motion is finished
    if (m_step_count >= m_p1 + m_p2 + m_p3) {
      m_finished = true;
      v = m_v_end;  // Final velocity
    }

    return m_dir * v;  // Apply direction to velocity
  }

  // Reset the motion profile
  void reset() {
    m_step_count = 0;
    m_finished = true;
  }

  // Check if the motion profile is complete
  inline bool isFinished() const {
    return m_finished;
  }

 private:
  // Member variables
  float m_distance;   // Total distance to travel
  float m_dir;        // Direction of motion (+1 or -1)
  float m_v_start;    // Starting velocity
  float m_v_max;      // Maximum velocity
  float m_v_end;      // Ending velocity
  float m_accel;      // Acceleration value
  float m_deltaTime;  // Time step size

  int m_p1;          // Steps in acceleration phase
  int m_p2;          // Steps in constant velocity phase
  int m_p3;          // Steps in deceleration phase
  int m_step_count;  // Current step count

  bool m_finished;  // Flag to indicate motion completion

  // Helper function to determine the sign of a number
  inline float sign(float a) const {
    return a >= 0 ? 1.0f : -1.0f;
  }
};

#endif /* CONTROLLER_TRAPEZOID_H_ */
