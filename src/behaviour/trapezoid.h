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
 * I found the original code for this trapezoidal profiler as part of the
 * repository for MIZUHO, a micromouse by user 12312.
 *
 * https://github.com/idt12312/MIZUHO
 *
 * It has been refactored and edited to match more closely with my derivation
 * of the equations which can be found in the docs directory
 */
class Trapezoid {
 public:
  // Default constructor initializes the trapezoid profile to zero distance and velocities
  Trapezoid()
      : m_s(0),           // Total distance to travel
        m_v1(0),          // Starting velocity
        m_v_limit(0),     // Maximum velocity
        m_v2(0),          // Ending velocity
        m_a(0),           // Acceleration value
        m_dt(0.001f),     // Time step size
        m_dir(0),         // Direction of motion (+1 or -1)
        m_step_count(0),  // Step counter for velocity progression
        m_finished(true)  // Motion state flag
  {
  }

  // Parameterized constructor initializes the trapezoid motion profile
  Trapezoid(float dist, float v_start, float v_max, float v_end, float accel, float dt = 0.001f)
      : m_s(std::abs(dist)),         // Absolute distance to ensure positive magnitude
        m_v1(std::abs(v_start)),     // Ensure positive velocity
        m_v_limit(std::abs(v_max)),  // Ensure positive max velocity
        m_v2(std::abs(v_end)),       // Ensure positive end velocity
        m_a(std::abs(accel)),        // Ensure positive acceleration
        m_dt(dt),                    // Time step size
        m_dir(sign(dist)),           // Direction of motion (+1 or -1)
        m_step_count(0),             // Initialize step count
        m_finished(true)             // Start with motion as finished
  {
    init();
  }

  virtual ~Trapezoid() = default;

  void begin() {
    m_step_count = 0;
    m_finished = false;
  }

  // Initialize the motion profile
  // TODO what if Vmax < V1 or V2?
  void init() {
    // Calculate the steps required for acceleration and deceleration phases (p1 and p3)
    m_p1 = static_cast<int>(-(2 * m_v1 - std::sqrt(2 * m_v1 * m_v1 + 2 * m_v2 * m_v2 + 4 * m_s * m_a)) / (2 * m_a * m_dt));
    m_p3 = static_cast<int>(-(2 * m_v2 - std::sqrt(2 * m_v1 * m_v1 + 2 * m_v2 * m_v2 + 4 * m_s * m_a)) / (2 * m_a * m_dt));

    // Recalculate acceleration to ensure distance matches the provided input
    m_a = (m_s - m_dt * ((float)m_p1 * m_v1 + (float)m_p3 * m_v2)) /  //
          (0.5f * m_dt * m_dt * (float)(m_p1 * m_p1 + m_p3 * m_p3));

    // Check if maximum velocity is reached during acceleration
    if (m_v1 + m_a * (float)m_p1 * m_dt < m_v_limit) {
      m_p2 = 0;  // No constant velocity phase
    } else {
      // Adjust acceleration and deceleration phases to reach maximum velocity
      m_p1 = static_cast<int>((m_v_limit - m_v1) / (m_dt * m_a));
      m_p3 = static_cast<int>((m_v_limit - m_v2) / (m_dt * m_a));

      float t1 = (float)m_p1 * m_dt;
      float t3 = (float)m_p3 * m_dt;
      // Calculate the remaining distance for the constant velocity phase
      float cruise_dist = m_s - (0.5f * m_a * (t1 * t1 + t3 * t3) + m_v1 * t1 + m_v2 * t3);

      // Calculate the number of steps at maximum velocity
      m_p2 = static_cast<int>(cruise_dist / (m_dt * m_v_limit));

      // Adjust the actual maximum velocity for exact result
      m_v_limit = cruise_dist / (m_dt * (float)m_p2);
    }

    // Reset step count and set motion as active
    m_step_count = 0;
    m_finished = true;
  }

  // Calculate the next velocity step in the profile
  float next() {
    float v = 0.0f;
    if (m_step_count <= m_p1) {  // Phase 1: Acceleration
      v = m_v1 + m_a * m_dt * (float)m_step_count;
    } else if (m_step_count <= m_p1 + m_p2) {  // Phase 2: Constant velocity
      v = m_v_limit;
    } else if (m_step_count < m_p1 + m_p2 + m_p3) {  // Phase 3: Deceleration
      v = m_v2 + m_a * m_dt * float(m_p1 + m_p2 + m_p3 - m_step_count);
    }

    // Increment step count if motion is not finished
    if (!m_finished) {
      m_step_count++;
    }

    // Check if motion is finished
    if (m_step_count >= m_p1 + m_p2 + m_p3) {
      m_finished = true;
      v = m_v2;  // Final velocity
    }

    return m_dir * v;  // Apply direction to velocity
  }

  // Reset the motion profile
  void reset() {
    m_step_count = 0;
    m_finished = true;
  }

  /***
   * Calculates the time needed for this profile
   * NEVER call this while the profile is active
   * @return time in seconds
   */
  float get_duration() {
    m_step_count = 0;
    m_finished = false;
    while (!m_finished) {
      next();
    }
    return m_dt * (float)m_step_count;
  }

  // Check if the motion profile is complete
  inline bool isFinished() const {
    return m_finished;
  }

 private:
  // Member variables
  float m_s;        // Total distance to travel
  float m_v1;       // Starting velocity
  float m_v_limit;  // Maximum velocity
  float m_v2;       // Ending velocity
  float m_a;        // Acceleration value
  float m_dt;       // Time step size

  float m_dir;       // Direction of motion (+1 or -1)
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
