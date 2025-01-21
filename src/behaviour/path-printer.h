//
// Created by peter on 09/01/25.
//

#pragma once
#include <stdio.h>

#include "actions.h"

void print_action(Action action) {
  if (action == OP_END) {
    printf("OP_END    ");
  } else if (action == OP_ERROR) {
    printf("OP_ERROR, ");
  } else if (action == OP_EXPLORE) {
    printf("OP_EXP    ");
  } else if (action == OP_STOP) {
    printf("STOP   ");
  } else if (action == OP_BEGIN) {
    printf("OP_BEG    ");
  } else if (action == OP_BEGIN_HS) {
    printf("OP_BEG_HS ");
  } else if (action.is_ortho_move()) {
    printf("FWD%d, ", action.op_code - OP_TYPE_ORTHO);
  } else if (action.is_diagonal_move()) {
    printf("DIA%d, ", action.op_code - OP_TYPE_DIAG);
  } else if (action.is_inplace_turn()) {
    printf("%s, ", inPlaceTurnNames[action.op_code - OP_TURN_INPLACE]);
  } else if (action.is_smooth_turn()) {
    printf("%s, ", smoothTurnNames[action.op_code - OP_TURN_SMOOTH]);
  } else if (action >= OP_ERR_BASE) {
    printf("ERR_%02d, ", action.op_code - OP_ERR_BASE);
  } else {
    printf("%6d, ", action.op_code);
    //    printf("?? %02x, ", action.op_code);
  }
}

void print_action_list(Action *action_list) {
  char done = 0;
  printf("\n");
  while (!done) {
    print_action(*action_list);
    if (*action_list == OP_STOP) {
      done = true;
    }
    action_list++;
  }
}
