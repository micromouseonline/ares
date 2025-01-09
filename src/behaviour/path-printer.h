//
// Created by peter on 09/01/25.
//

#pragma once
#include <stdio.h>

#include "motion-commands.h"

void print_operation(MotionCommand operation) {
  if (operation == OP_END) {
    printf("OP_END    ");
  } else if (operation == OP_ERROR) {
    printf("OP_ERROR, ");
  } else if (operation == OP_EXPLORE) {
    printf("OP_EXP    ");
  } else if (operation == OP_STOP) {
    printf("STOP   ");
  } else if (operation == OP_BEGIN) {
    printf("OP_BEG    ");
  } else if (operation == OP_BEGIN_HS) {
    printf("OP_BEG_HS ");
  } else if (operation.is_ortho_move()) {
    printf("FWD%d, ", operation.op_code - OP_TYPE_ORTHO);
  } else if (operation.is_diagonal_move()) {
    printf("DIA%d, ", operation.op_code - OP_TYPE_DIAG);
  } else if (operation.is_inplace_turn()) {
    printf("%s, ", inPlaceTurnNames[operation.op_code - OP_TURN_INPLACE]);
  } else if (operation.is_smooth_turn()) {
    printf("%s, ", smoothTurnNames[operation.op_code - OP_TURN_SMOOTH]);
  } else if (operation >= OP_ERR_BASE) {
    printf("ERR_%02d, ", operation.op_code - OP_ERR_BASE);
  } else {
    printf("%6d, ", operation.op_code);
    //    printf("?? %02x, ", operation.op_code);
  }
}

void print_operations_list(MotionCommand *op_list) {
  char done = 0;
  printf("\n");
  while (!done) {
    print_operation(*op_list);
    if (*op_list == OP_STOP) {
      done = true;
    }
    op_list++;
  }
}
