//
// Created by peter on 09/01/25.
//

#pragma once
#include <stdio.h>

#include "actions.h"
#include "trajectories/cubic.h"
#include "trajectories/cubic_parameters.h"
#include "trajectories/spinturn.h"
#include "trajectories/straight.h"
#include "trajectory.h"
/// Don't move this - there will be more features later
/// for example, print with durations

inline void print_action_list(Action* action_list) {
  char done = 0;
  printf("\n");
  while (!done) {
    printf("%s", (*action_list).name());
    if (*action_list == OP_STOP) {
      done = true;
    }
    action_list++;
  }
}

inline float cubic_calculate_speed(const CubicTurnParameters& params, float acceleration) {
  float speed = sqrtf(acceleration * 4.0f * params.length / (6.0f * fabsf(params.angle * RADIANS)));
  return speed;
}

inline float printActionWithCost(Action& act, Action* previous, Action* next, Pose& start_pose, Pose& end_pose) {
  float vMax = 5000;
  float acc = 14000;
  float duration = 0;
  //  std::unique_ptr<Trajectory> traj;
  Trajectory* traj;
  if (act.is_straight_move()) {
    float one_cell = 180.0f;
    if (act.is_diagonal_straight()) {
      one_cell = 180.0f * 0.7071;
    }
    float dist = act.length() * one_cell;
    float start_speed = start_pose.getVelocity();
    float end_speed = 0;
    if (previous && previous->is_smooth_turn()) {
      int p_type = previous->get_smooth_turn_type();
      dist -= cubic_params[p_type].out_offset;
    }
    if (next && next->is_smooth_turn()) {
      int p_type = next->get_smooth_turn_type();
      CubicTurnParameters params = cubic_params[p_type];
      dist -= params.in_offset;
      float speed = cubic_calculate_speed(params, acc);
      // but do not exceed the maximum permitted for this turn.
      end_speed = speed;
    }
    if (next->op_code == ACT_END) {
      end_speed = 0;
    }
    dist = std::max(1.0f, dist);
    //    traj = std::make_unique<Straight>(dist, start_speed, vMax, end_speed, acc);
    straightTraj = Straight(dist, start_speed, vMax, end_speed, acc);
    traj = &straightTraj;

    traj->init(start_pose);
    duration = traj->get_duration();
  } else if (act.is_smooth_turn()) {
    int type = act.get_smooth_turn_type();
    float length = cubic_params[type].length;
    float angle = cubic_params[type].angle;
    float speed = cubic_params[type].speed_max;
    //    traj = std::make_unique<Cubic>(length, angle, speed);
    cubicTraj = Cubic(length, angle, speed);
    traj = &cubicTraj;
    traj->init(Pose());
    duration = traj->get_duration();
  } else if (act.is_spin_turn()) {
    float angle = act.spinTurnAngle();
    //    traj = std::make_unique<Spinturn>(angle, 0, 600, 0, 40000);
    spinTurnTraj = Spinturn(angle, 0, 600, 0, 40000);
    traj = &spinTurnTraj;
    traj->init(start_pose);
    duration = traj->get_duration();
  } else {
    //    traj = std::make_unique<IdleTrajectory>();
    traj = &idleTraj;
    traj->init(start_pose);
    duration = traj->get_duration();
  }
  end_pose = traj->getCurrentPose();
  printf("%3d %s %8.1f mm %6.3f s\n", act.op_code, act.name(), end_pose.getDistance(), traj->get_duration());
  return duration;
}
//
// const uint8_t test_path[] = {FWD15, SS90FR, FWD15, SD135R, DIA2, DS45L, FWD13, SS180R, FWD3, SD135L, DIA4, DS45R,  FWD2,  SD135R, DIA8, DS45L,
//                             FWD7,  SS90FL, FWD2,  SD135L, DIA2, DD90R, DIA2,  DD90L,  DIA2, DS45R,  FWD2, SD45R,  DIA10, DD90L,  DIA2, DS135R,
//                             FWD2,  SD45R,  DIA2,  DD90L,  DIA2, DS45R, FWD2,  SD135R, DIA2, DS45L,  FWD3, SS90FR, FWD4,  SS90FL, FWD2, ACT_END};

const uint8_t test_path[] = {FWD2, SS90FR, FWD2, ACT_END};

const int test_count = sizeof(test_path);
inline void printActionListWithCost(const uint8_t* path) {
  Pose start;
  Pose end;
  float duration = 0.0f;
  float distance = 0;

  for (int i = 0; i < test_count; i++) {
    Action act(path[i]);

    Action prev(0);
    Action next(0);
    if (i > 0 && act.is_straight_move()) {
      prev = Action(path[i - 1]);
    }
    if (i < test_count - 1) {
      next = Action(path[i + 1]);
    } else {
      end = Pose();
    }

    duration += printActionWithCost(act, &prev, &next, start, end);
    distance += end.getDistance();
    start = end;
  }
  printf("Total distance = %.1f mm in %5.3f s\n", distance, duration);
}

inline void listActionsWithCosts() {
  for (int x = 0; x < 255; x++) {
    Action act(x);
    float vMax = 5000;
    float acc = 10000;
    float duration = 0;
    float dist = 0;
    std::unique_ptr<Trajectory> traj;
    if (act.is_straight_move()) {
      float one_cell = 180.0f;
      if (act.is_diagonal_straight()) {
        one_cell = 180.0f * 0.7071;
      }
      dist = act.length() * one_cell;
      traj = std::make_unique<Straight>(dist, 0, vMax, 0, acc);
      traj->init(Pose());
      duration = traj->get_duration();
    } else if (act.is_smooth_turn()) {
      int type = act.get_smooth_turn_type();
      float length = cubic_params[type].length;
      float angle = cubic_params[type].angle;
      float speed = cubic_params[type].speed_max;
      traj = std::make_unique<Cubic>(length, angle, speed);
      traj->init(Pose());
      duration = traj->get_duration();
    } else if (act.is_spin_turn()) {
      float angle = act.spinTurnAngle();
      traj = std::make_unique<Spinturn>(angle, 0, 600, 0, 40000);
      traj->init(Pose());
      duration = traj->get_duration();
    } else {
      duration = 0.0f;
    }
    if (duration > 0) {
      std::cout << x << " : " << act.name() << "  " << dist << "   " << duration << std::endl;
    }
  }
}
