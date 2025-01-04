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

#pragma once
#include <cmath>

/***
 * The Trajectory class is the base class for all motion profiles.
 * It is a pure virtual class that cannot be instantiated - it
 * provides a common type and basic interface for various
 * trajectory types that inherit from it.
 *
 * The primary function of this class is to provide a series of
 * forward and angular velocities that will move the robot
 * from the start pose to the end pose.
 *
 * Typical usage involves constructing a derived class of Trajectory,
 * initializing it, and then repeatedly calling the next() method
 * to retrieve a structure containing the two velocities.
 *
 * Check isFinished() to determine when the trajectory is complete.
 *
 * Utility methods are available to retrieve the duration, the pose
 * at any time, and the final state.
 *
 * Be cautious when using some utilities while the trajectory
 * is active, as this may affect performance or accuracy.
 *
 * Derived classes have custom constructors designed for
 * specific types of motion, such as linear moves or
 * in-place/curved turns.
 *
 */

#include "common/pose.h"

class Trajectory {
 public:
  Trajectory() : m_start_pose(), m_current_pose(), m_delta_time(0.001f), m_current_step(0), m_finished(true) {};

  virtual ~Trajectory() = default;

  // Initialize the motion profile
  virtual void init(const Pose& pose) {
    m_start_pose = pose;
    m_current_pose = pose;
  };

  // set the control variables to the start of the trajectory, ready for stepping through
  virtual void begin() {
    m_current_step = 0;
    m_finished = false;
  };

  // Calculate the next velocity step in the profile
  virtual float next() {
    if (m_current_step >= 100) {
      m_finished = true;
      return 0.0f;
    }
    m_current_step++;
    float v = 10.0f;
    m_current_pose.setVelocity(v);
    m_current_pose.advance(m_delta_time);
    return v;
  }

  int getCurrentStep() const {
    return m_current_step;
  }

  Pose getCurrentPose() const {
    return m_current_pose;
  }

  /***
   * Do not call this on an active profile.
   * @param t
   * @return
   */
  virtual Pose getPoseAtTime(const float t) {
    if (!m_finished) {
      return m_start_pose;
    }
    m_current_step = 0;
    while ((float)m_current_step * m_delta_time < t) {
      next();
    }
    Pose result = m_current_pose;
    m_current_pose = m_start_pose;
    return result;
  }

  virtual Pose getFinalPose() {
    if (!m_finished) {
      return m_start_pose;
    }
    get_duration();  // steps through to the end of the traectory
    Pose result = m_current_pose;
    m_current_pose = m_start_pose;
    return result;
  }

  /***
   * All trajectories start with a defined pose. They proceed
   * by stepwise advancement and a count is kept of the current
   * step. When a profile is no longer active it sets a flag
   * to be true.
   * @return - nothing
   */
  virtual void reset() {
    m_current_pose = m_start_pose;
    m_current_step = 0;
    m_finished = true;
  }

  /***
   * Calculates the time needed for this profile.
   * You should not call this while the profile is active.
   * If you dou, it will simply return 0;
   * @return time in seconds
   *
   * QUERY: does this ever really need to be overridden?
   */
  virtual float get_duration() {
    if (!m_finished) {
      return 0;
    }
    m_current_step = 0;
    m_finished = false;
    while (!m_finished) {
      next();
    }
    return m_delta_time * (float)m_current_step;
  };

  // Check if the motion profile is complete
  bool isFinished() const {
    return m_finished;
  }

  void setDeltaTime(float dt) {
    m_delta_time = dt;
  }

  [[nodiscard]] float getDeltaTime() const {
    return m_delta_time;
  }

 private:
 protected:
  Pose m_start_pose;
  Pose m_current_pose;
  float m_delta_time;
  float m_current_step;
  bool m_finished;  // Flag to indicate motion completion
};
