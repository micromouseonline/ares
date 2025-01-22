//
// Created by peter on 09/01/25.
//

#pragma once
#include <stdio.h>

#include "actions.h"

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
