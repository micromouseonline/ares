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
  Trapezoid()
      : m_distance(0),       //
        m_dir(0),            //
        m_v_start(0),        //
        m_v_max(0),          //
        m_v_end(0),          //
        m_deltaTime(0.001f)  //
  {
    reset();
  };

  Trapezoid(float dist, float v_start, float v_max, float v_end, float accel, float dt = 0.001f)
      : m_distance(std::abs(dist)),    //
        m_dir(sign(dist)),             //
        m_v_start(std::abs(v_start)),  //
        m_v_max(std::abs(v_max)),      //
        m_v_end(std::abs(v_end)),      //
        m_acceleration(std::abs(accel)),
        m_deltaTime(dt)  //
  {
  }

  virtual ~Trapezoid() = default;

  void begin() {
    m_p1 = (-(2 * m_v_start - std::sqrt(2 * m_v_start * m_v_start + 2 * m_v_end * m_v_end + 4 * m_distance * m_acceleration)) /
            (2 * m_acceleration * m_deltaTime));
    m_p3 =
        (-(2 * m_v_end - std::sqrt(2 * m_v_start * m_v_start + 2 * m_v_end * m_v_end + 4 * m_distance * m_acceleration)) / (2 * m_acceleration * m_deltaTime));

    m_acceleration = (m_distance - m_deltaTime * (m_p1 * m_v_start + m_p3 * m_v_end)) / (0.5 * m_deltaTime * m_deltaTime * (m_p1 * m_p1 + m_p3 * m_p3));

    if (m_v_start + m_acceleration * m_p1 * m_deltaTime < m_v_max) {
      m_p2 = 0;
    } else {
      m_p1 = int((m_v_max - m_v_start) / (m_deltaTime * m_acceleration));
      m_p3 = int((m_v_max - m_v_end) / (m_deltaTime * m_acceleration));

      const float L2 = m_distance - (0.5 * m_acceleration * ((m_p1 * m_deltaTime) * (m_p1 * m_deltaTime) + (m_p3 * m_deltaTime) * (m_p3 * m_deltaTime)) +
                                     m_v_start * m_p1 * m_deltaTime + m_v_end * m_p3 * m_deltaTime);
      m_p2 = (size_t)(L2 / (m_deltaTime * m_v_max));
      m_v_max = L2 / (m_deltaTime * m_p2);
    }

    m_step_count = 0;
    m_finished = false;
  }
  float next() {
    float v;
    if (m_step_count <= m_p1) {
      v = m_v_start + m_acceleration * m_deltaTime * m_step_count;
    } else if (m_step_count <= m_p1 + m_p2) {
      v = m_v_max;
    } else if (m_step_count < m_p1 + m_p2 + m_p3) {
      v = m_v_end + m_acceleration * m_deltaTime * (m_p1 + m_p2 + m_p3 - m_step_count);
    }

    if (!m_finished) {
      m_step_count++;
    }

    if (m_step_count >= m_p1 + m_p2 + m_p3) {
      m_finished = true;
      v = m_v_end;
    }
    return m_dir * v;
  }

  void reset() {
    m_step_count = 0;
    m_finished = true;
  }

  inline bool isFinished() const {
    return m_finished;
  }

 private:
  float m_distance;
  float m_dir;
  float m_v_start;
  float m_v_max;
  float m_v_end;
  float m_acceleration;
  float m_deltaTime;

  int m_p1;
  int m_p2;
  int m_p3;
  int m_step_count;

  bool m_finished = true;

  inline float sign(float a) const {
    return a >= 0 ? 1 : -1;
  }
};

#endif /* CONTROLLER_TRAPEZOID_H_ */
