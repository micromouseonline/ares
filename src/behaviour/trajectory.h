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

#ifndef BEHAVIOUR_TRAJECTORY_H
#define BEHAVIOUR_TRAJECTORY_H

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
  virtual ~Trajectory() = default;

  // Initialize the motion profile
  virtual void init(const Pose) = 0;

  // set the control variables to the start of the trajectory, ready for stepping through
  virtual void begin() = 0;

  // Calculate the next velocity step in the profile
  virtual float next() = 0;

  virtual Pose getPoseAtTime(float t) = 0;

  virtual Pose getFinalPose() = 0;

  virtual  // Reset the motion profile
      void
      reset() {
  }

  // calculate the time taken, in seconds, to execute the trajectory
  virtual float get_duration() = 0;

  // Check if the motion profile is complete
  bool isFinished() const {
    return m_finished;
  }

  void setStartPose(Pose p) {
    m_start_pose = p;
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
  Trajectory() : m_start_pose(), m_current_pose(), m_delta_time(0.001f), m_current_step(0), m_finished(true) {};
};

#endif /* BEHAVIOUR_TRAJECTORY_H */
