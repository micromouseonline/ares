/*
 * Trapezoid.h
 *
 *  Created on: 2016/11/04
 *      Author: idt12312
 */

#ifndef CONTROLLER_TRAPEZOID_H_
#define CONTROLLER_TRAPEZOID_H_

#include <cstddef>

// Acceleration(Absolute value)
// Maximum
// m_finished point
class Trapezoid {
 public:
  Trapezoid(float dist, float v_start, float v_max, float v_end, float accel, float dt = 0.001f)
      : m_distance(std::abs(dist)),    //
        m_dir(sign(dist)),             //
        m_v_start(std::abs(v_start)),  //
        m_v_max(std::abs(v_max)),      //
        m_v_end(std::abs(v_end)),      //
        m_deltaTime(dt)                //
  {
    accel = std::abs(accel);
    m_phase1_steps = (-(2 * m_v_start - std::sqrt(2 * m_v_start * m_v_start + 2 * m_v_end * m_v_end + 4 * m_distance * accel)) / (2 * accel * m_deltaTime));
    m_phase3_steps = (-(2 * m_v_end - std::sqrt(2 * m_v_start * m_v_start + 2 * m_v_end * m_v_end + 4 * m_distance * accel)) / (2 * accel * m_deltaTime));

    m_acceleration = (m_distance - m_deltaTime * (m_phase1_steps * m_v_start + m_phase3_steps * m_v_end)) /
                     (0.5 * m_deltaTime * m_deltaTime * (m_phase1_steps * m_phase1_steps + m_phase3_steps * m_phase3_steps));

    if (m_v_start + m_acceleration * m_phase1_steps * m_deltaTime < v_max) {
      m_phase2_steps = 0;
    } else {
      m_phase1_steps = (size_t)((m_v_max - m_v_start) / (m_deltaTime * m_acceleration));
      m_phase3_steps = (size_t)((m_v_max - m_v_end) / (m_deltaTime * m_acceleration));

      const float L2 =
          m_distance -
          (0.5 * m_acceleration *
               ((m_phase1_steps * m_deltaTime) * (m_phase1_steps * m_deltaTime) + (m_phase3_steps * m_deltaTime) * (m_phase3_steps * m_deltaTime)) +
           m_v_start * m_phase1_steps * m_deltaTime + m_v_end * m_phase3_steps * m_deltaTime);
      m_phase2_steps = (size_t)(L2 / (m_deltaTime * m_v_max));
      m_v_max = L2 / (m_deltaTime * m_phase2_steps);
    }

    reset();
  }

  virtual ~Trapezoid() = default;

  float next() {
    float v;
    if (m_step_count <= m_phase1_steps) {
      v = m_v_start + m_acceleration * m_deltaTime * m_step_count;
    } else if (m_step_count <= m_phase1_steps + m_phase2_steps) {
      v = m_v_max;
    } else if (m_step_count < m_phase1_steps + m_phase2_steps + m_phase3_steps) {
      v = m_v_end + m_acceleration * m_deltaTime * (m_phase1_steps + m_phase2_steps + m_phase3_steps - m_step_count);
    }

    if (!m_finished)
      m_step_count++;

    if (m_step_count >= m_phase1_steps + m_phase2_steps + m_phase3_steps) {
      m_finished = true;
      v = m_v_end;
    }
    return m_dir * v;
  }

  void reset() {
    m_step_count = 0;
    m_finished = false;
  }

  inline bool is_end() const {
    return m_finished;
  }

 private:
  const float m_distance;
  const float m_dir;
  const float m_v_start;
  float m_v_max;
  const float m_v_end;
  float m_acceleration;
  const float m_deltaTime;

  int m_phase1_steps;
  int m_phase2_steps;
  int m_phase3_steps;
  int m_step_count;

  bool m_finished;

  inline float sign(float a) const {
    return a >= 0 ? 1 : -1;
  }
};

#endif /* CONTROLLER_TRAPEZOID_H_ */
