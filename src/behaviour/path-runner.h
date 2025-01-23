//
// Created by peter on 23/01/25.
//

#pragma once

#include "actions.h"
#include "common/pose.h"
#include "trajectories/cubic.h"
#include "trajectories/spinturn.h"
#include "trajectories/straight.h"
#include "trajectory.h"
/***
 * Pathrunner will perform operations on a path described as an
 * array of Actions PpCodes. This path will have been generated from a
 * path string and describes a number of actions the robot will
 * perform. For example:
 *    const uint8_t test_path[] = {FWD2, SS90FR, FWD2, ACT_END};
 * While the normal use will be to cause the vehicle to move
 * along a connected series of trajectories, the list can also
 * be traversed to get the length and duration of the path as
 * well as the final Pose

 */
class PathRunner {
 public:
  float m_max_velocity = 100;
  float m_max_omega = 100;
  float m_accel = 1000;
  float m_alpha = 1000;

  void setMaximumVelocity(float v_max) {
    m_max_velocity = v_max;
  }
  void setMaximumOmega(float w_max) {
    m_max_velocity = w_max;
  }
  void setMaximumSpeeds(float v, float w) {
    setMaximumVelocity(v);
    setMaximumOmega(w);
  }

  void setAccel(float accel) {
    m_accel = accel;
  }
  void setAlpha(float alpha) {
    m_alpha = alpha;
  }
  void setAccelerations(float v_dot, float w_dot) {
    setAccel(v_dot);
    setAlpha(w_dot);
  }

  Pose executeAction(const Action& action, const Action& previous, const Action& next, const Pose& start_pose) {
    if (action.op_code == ACT_END) {
      return start_pose;
    } else if (action.op_code == ACT_BEGIN) {
      return start_pose;
    } else if (action.is_straight_move()) {
      float one_cell = 180.0f;
      if (action.is_diagonal_straight()) {
        one_cell = 180.0f * 0.7071;
      }
      float dist = action.length() * one_cell;
      float v_start = start_pose.getVelocity();
      float v_end = start_pose.getVelocity();
      float start_speed = start_pose.getVelocity();
      float end_speed = 0;
      if (previous.op_code != ACT_BEGIN && previous.is_smooth_turn()) {
        int p_type = previous.get_smooth_turn_type();
        dist -= cubic_params[p_type].out_offset;
      }
      if (next.op_code != ACT_END && next.is_smooth_turn()) {
        int p_type = next.get_smooth_turn_type();
        CubicTurnParameters params = cubic_params[p_type];
        dist -= params.in_offset;
        float speed = cubic_calculate_speed(params, m_accel);
        // but do not exceed the maximum permitted for this turn.
        end_speed = speed;
      }
      if (next.op_code == ACT_END) {
        end_speed = 0;
      }
      dist = std::max(1.0f, dist);
      straightTraj = Straight(dist, start_speed, m_max_velocity, end_speed, m_accel);
      Trajectory* trajectory = &straightTraj;

      trajectory->init(start_pose);
      float duration = trajectory->getDuration();
      return trajectory->getCurrentPose();
    }
    /// Deliberately return an invalid pose to signale an error
    /// Tacky - I know.
    Pose bad_pose;
    bad_pose.setDistance(-INT16_MAX);
    return bad_pose;
  }

  inline Pose executeActionList(const uint8_t* actions, const Pose& start_pose) {
    int length = strlen((char*)actions);
    Trajectory* trajectory;
    Pose current_pose = start_pose;
    for (int i = 0; i < length; i++) {
      Action prev = Action(ACT_END);
      if (i > 0) {
        prev = Action(actions[i - 1]);
      }
      Action next = Action(actions[i + 1]);
      current_pose = executeAction(Action(actions[i]), prev, next, current_pose);
    }
    return current_pose;
  }

  inline float printActionWithCost(Action& act, Action* previous, Action* next, Pose& start_pose, Pose& end_pose) {
    float vMax = 5000;
    float acc = 14000;
    float duration = 0;
    //  std::unique_ptr<Trajectory> traj;
    Trajectory* traj;
    if (act.is_straight_move()) {
    } else if (act.is_smooth_turn()) {
      int type = act.get_smooth_turn_type();
      float length = cubic_params[type].length;
      float angle = cubic_params[type].angle;
      float speed = cubic_params[type].speed_max;
      cubicTraj = Cubic(length, angle, speed);
      traj = &cubicTraj;
      traj->init(Pose());
      duration = traj->getDuration();
    } else if (act.is_spin_turn()) {
      float angle = act.spinTurnAngle();
      spinTurnTraj = Spinturn(angle, 0, 600, 0, 40000);
      traj = &spinTurnTraj;
      traj->init(start_pose);
      duration = traj->getDuration();
    } else {
      traj = &idleTraj;
      traj->init(start_pose);
      duration = traj->getDuration();
    }
    end_pose = traj->getCurrentPose();
    printf("%3d %s %8.1f mm %6.3f s\n", act.op_code, act.name(), end_pose.getDistance(), traj->getDuration());
    return duration;
  }

 private:
};
