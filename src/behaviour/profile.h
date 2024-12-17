//
// Created by peter on 12/12/24.
//

#ifndef PROFILE_H
#define PROFILE_H

#include <cmath>

/***
 * The Profile class manages speed as a function of time or distance. A profile
 * is trapezoidal in shape and consists of up to three phases.
 *
 *  - Acceleration : changing from the initial speed to the maximum speed
 *  - Coasting     : moving at the maximum speed
 *  - Braking      : changing to the final speed
 *
 * The names are only really relevant to a subset of all possible profiles since
 * the coasting section may not be possible if the acceleration is low or the
 * maximum speed is otherwise unreachable. Also, note that the initial speed may
 * be higher than the maximum speed so the first phase actually needs braking.
 *
 * Once started, the profile must be regularly updated from the system control
 * loop and this must be done before the motor control task since it uses the
 * current profile speed in its calculation.
 *
 * The robot has two instances of a Profile - one for forward motion and one
 * for rotation. These are completely independent but the combination of the
 * two allows the robot full freedom of motion within the constraints imposed
 * by the drive arrangement. That is, any combination of forward and rotary
 * motion is possible at all times.
 *
 * Although the units in the comments are shown as mm, the class is unit
 * agnostic and you can interpret the units as mm, cm, deg, bananas or anything
 * else. Just be consistent when using speed and acceleration.
 */
constexpr float MIN_POSITION_TOLERANCE = 0.125f;  // mm or deg
constexpr float MIN_VELOCITY_FOR_ZERO = 5.0f;     // mm/s or deg/s

class Profile {
 public:
  enum State {
    PS_IDLE = 0,
    PS_ACCELERATING = 1,
    PS_BRAKING = 2,
    PS_FINISHED = 3,
  };

  void reset() {
    m_position = 0;
    m_speed = 0;
    m_target_speed = 0;
    m_state = PS_IDLE;
  }

  bool isFinished() {
    return m_state == PS_FINISHED;  //
  }

  /// @brief  Begin a profile. Once started, it will automatically run to completion
  ///         Subsequent calls before completion supercede all the parameters.
  ///         Called may monitor progress using is_finished() method
  /// @param distance     (mm)     always positive
  /// @param top_speed    (mm/s)   negative values move the robot in reverse
  /// @param final_speed  (mm/s)
  /// @param acceleration (mm/s/s)

  void start(float distance, float top_speed, float final_speed, float acceleration) {
    top_speed = fabsf(top_speed);
    final_speed = fabsf(final_speed);
    m_sign = (distance < 0) ? -1 : +1;
    if (distance < 0) {
      distance = -distance;
    }
    if (distance < 1.0) {
      m_state = PS_FINISHED;
      return;
    }
    if (final_speed > top_speed) {
      final_speed = top_speed;
    }

    m_position = 0;
    m_final_position = distance;
    m_target_speed = m_sign * top_speed;
    m_final_speed = m_sign * final_speed;
    m_acceleration = fabsf(acceleration);
    /// pre-calculate for better performance in the update method
    if (m_acceleration > 1.0f) {
      m_one_over_acc = 1.0f / m_acceleration;
    } else {
      m_one_over_acc = 1.0;
    }
    m_state = PS_ACCELERATING;
  }

  /// @brief causes the profile to immediately terminate with the speed to zero
  ///        note that even when the state is PS_FINISHED, the profiler will
  ///        continue to try and reach the target speed. (zero in this case)
  void stop() {
    m_target_speed = 0;
    finish();
  }

  /// @brief  Force a profile to finish with the target speed set to the final speed
  void finish() {
    m_speed = m_target_speed;
    m_state = PS_FINISHED;
  }

  void set_state(State state) {
    m_state = state;  //
  }

  /// @brief  Calculate the distance needed to get to the final speed from the
  ///         current speed using the current acceleration.
  /// @return distance (mm)
  float get_braking_distance() {
    return fabsf(m_speed * m_speed - m_final_speed * m_final_speed) * 0.5 * m_one_over_acc;  //
  }

  /// @brief  gets the distance travelled (mm) since the last call to start(). If there
  ///         was a prior call to setPosition() distance is incremented from there.
  /// @return distance travelled (mm)
  float position() {
    return m_position;
  }

  float getSpeed() {
    return m_speed;  //
  }

  float acceleration() {
    return m_acceleration;
  }

  /***
   * @brief Sets a target speed that the profiler will try to reach using the supplied acceleration
   * Leave the acceleration at zero to immediately set the speed
   * @param speed
   */
  void setTargetSpeed(const float speed, const float acceleration = 0) {
    float a = fabsf(acceleration);
    if (a < kEpsilon) {
      m_speed = speed;
    } else {
      m_acceleration = a;
    }
    m_target_speed = speed;
  }

  // normally only used to alter position for forward error correction
  /// TODO: should I ever really need this?
  void adjustPosition(float adjustment) {
    m_position += adjustment;
  }

  void setPosition(float position) {
    m_position = position;
  }

  // update is called from within systick and should be safe from interrupts
  void update(float deltaTime) {
    if (m_state == PS_IDLE) {
      return;
    }
    float delta_v = m_acceleration * deltaTime;
    float remaining = fabsf(m_final_position) - fabsf(m_position);
    if (m_state == PS_ACCELERATING) {
      if (remaining < get_braking_distance()) {
        m_state = PS_BRAKING;
        if (fabsf(m_final_speed) < 0.01) {
          m_final_speed = 0.0f;
          // The magic number I normally set to be the same, numerically, as the current
          // acceleration. It is just a small velocity that ensures that the motion continues
          // past the finish point in case floating point rounding prevents that happening.
          // It is a nasty hack I keep meaning to find a more tidy solution for.
          m_target_speed = m_sign * 5.0f;  // magic number to make sure we reach zero
        } else {
          m_target_speed = m_final_speed;
        };
      }
    }
    // try to reach the target speed
    if (m_speed < m_target_speed) {
      m_speed += delta_v;
      if (m_speed > m_target_speed) {
        m_speed = m_target_speed;
      }
    }
    if (m_speed > m_target_speed) {
      m_speed -= delta_v;
      if (m_speed < m_target_speed) {
        m_speed = m_target_speed;
      }
    }
    // increment the position
    m_position += m_speed * deltaTime;
    // The number is a hack to ensure floating point rounding errors do not prevent the
    // loop termination. The units are mm and independent of the encoder resolution.
    // I figure that being within 1/8 of a mm will be close enough.
    if (m_state != PS_FINISHED && remaining < 0.125) {
      m_state = PS_FINISHED;
      m_target_speed = m_final_speed;
    }
  }

 private:
  int m_state = PS_IDLE;
  float m_speed = 0;
  float m_position = 0;
  int m_sign = 1;
  float m_acceleration = 0;
  float m_one_over_acc = 1;
  float m_target_speed = 0;
  float m_final_speed = 0;
  float m_final_position = 0;
};

#endif  // PROFILE_H
