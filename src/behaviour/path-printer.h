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

void print_action_list(Action* action_list) {
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

void listActionsWithCosts() {
  for (int x = 0; x < 255; x++) {
    Action act(x);
    float vMax = 5000;
    float acc = 10000;
    float duration = 0;
    if (act.is_straight_move()) {
      float one_cell = 180.0f;
      if (act.is_diagonal_straight()) {
        one_cell = 180.0f * 0.7071;
      }
      float dist = act.length() * one_cell;
      std::unique_ptr<Straight> traj = std::make_unique<Straight>(dist, 0, vMax, 0, acc);
      traj->init(Pose());
      act.setTrajectory(std::make_unique<Straight>(dist, 0, vMax, 0, acc));
      duration = act.getDuration();
    } else if (act.is_smooth_turn()) {
      int type = act.get_smooth_turn_type();
      float length = cubic_params[type].length;
      float angle = cubic_params[type].angle;
      float speed = cubic_params[type].speed_max;
      std::unique_ptr<Cubic> traj = std::make_unique<Cubic>(length, angle, speed);
      traj->init(Pose());
      act.setTrajectory(std::move(traj));
      duration = act.getDuration();
    } else if (act.is_spin_turn()) {
      float angle = act.spinTurnAngle();
      std::unique_ptr<Spinturn> traj = std::make_unique<Spinturn>(angle, 0, 600, 0, 40000);
      traj->init(Pose());
      act.setTrajectory(std::move(traj));
      duration = act.getDuration();
    } else {
      duration = 0.0f;
    }
    if (duration > 0) {
      std::cout << x << " : " << act.name() << "   " << duration << std::endl;
    }
  }
}
