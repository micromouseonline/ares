/***********************************************************************
 * Created by Peter Harrison on 15/12/2017.
 * Copyright (c) 2017 Peter Harrison
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without l> imitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 **************************************************************************/
#pragma once
#include <stdint.h>
#include "actions.h"

/*
 * Generate a action sequence from a string input. The generated path will
 * allow a micromouse to move diagonally in the maze.
 * The input string is a null-terminated array of characters.
 *
 * Passing in an empty string will generate an error.
 *
 * Only characters from the set FLRS are accepted. Other characters
 * in the input array will cause an error action to be emitted.
 *
 * The output is performed by the function *pActions++ = () which adds a single
 * action to the action list. Modify *pActions++ = () to do something else
 * with the generated results.
 *
 * If there is an error during conversion, the output action list will
 * still contain valid output up to the point where the error is detected.
 *
 * A single pass is taken through the input string and Actions are
 * generated as soon as there is an unambiguous state for the action.
 *
 * The input string will typically be generated form the maze solver data
 * and each valid character in that string has the following meaning:
 *   F : move forward one cell orthogonally
 *   R : perform an in-place right turn of 90 degrees
 *   L : perform an in-place left turn of 90 degrees
 *   S : Stop moving in this cell. This is the goal.
 *
 * Refer to the associated state chart for a view of how the states are
 * related to each other.
 *
 * NOTE that the output action list will be limited in size. The function
 * will continue to generate output values as long as there is valid input.
 * It knows nothing about the size of available output.
 *
 * The states have simple numbers rather than an enum since I could not
 * think of good names to use. Since then, I have but this version
 * maintains compatibility with the MINOS 2014 slides. Possible names are
 * shown as a comment for each state.
 */

class MotionCompiler {
 public:
  enum pathgen_state_t {
    PathInit,
    PathStart,
    PathOrtho_F,
    PathOrtho_R,
    PathOrtho_L,
    PathOrtho_RR,
    PathOrtho_LL,
    PathDiag_RL,
    PathDiag_LR,
    PathDiag_RR,
    PathDiag_LL,
    PathStop,
    PathExit,
    PathFinish
  };

  void makeInPlaceActions(const char *src, uint8_t *actions, uint16_t maxLength = 1024) {
    int p = 0;
    int runLength = 0;
    unsigned char cmd = OP_STOP;
    bool finished = false;
    assert(maxLength >= 2);
    while (!finished) {
      if (p >= maxLength) {
        actions[0] = ACT_ERROR;
        actions[1] = OP_STOP;
        break;
      }
      char c = *src++;

      switch (c) {
        case 'B':
          // action[p++] = OP_BEGIN;
          cmd = FWD0;
          runLength = 0;
          break;
        case 'F':
          cmd++;
          runLength++;
          if (runLength >= 31) {  // MAGIC: maximum for hald-size maze
            actions[p++] = ACT_ERROR;
            actions[p] = OP_STOP;
            finished = true;
          }
          break;
        case 'L':
          actions[p++] = cmd;
          actions[p++] = IP90L;
          cmd = FWD1;
          break;
        case 'R':
          actions[p++] = cmd;
          actions[p++] = IP90R;
          cmd = FWD1;
          break;
        case 'S':
          actions[p++] = cmd;
          actions[p++] = OP_STOP;
          finished = true;
          break;
        case 'X':
          actions[p++] = cmd;
          actions[p++] = ACT_EXPLORE;
          actions[p] = OP_STOP;
          finished = true;
          break;
        default:
          actions[p++] = ACT_ERROR;
          actions[p] = OP_STOP;
          finished = true;
          break;
      }
    };
  }

  /***
   * The smooth action list uses only orthogonal moves and 90 degree
   * explore turns. It should be very safe but continuous.
   * It is suitable for moving the mouse more rapidly in the maze while
   * exploring but where it is not felt safe to use diagonals
   *
   * This process does not really need a state machine but it is
   * here as a lead-in to the full diagonal path state machine.
   */
  void makeSmoothActions(const char *src, uint8_t *actions) {
    int runLength = 0;  // a counter for the number of cells to be crossed
    int p = 0;
    pathgen_state_t state = PathInit;
    while (state != PathFinish) {
      if (runLength >= 31) {  // MAGIC: maximum for half-size maze
        actions[p++] = ACT_ERROR;
        actions[p] = OP_STOP;
        break;
      }
      if (p >= NUM_ACTIONS_MAX) {
        actions[0] = ACT_ERROR;
        actions[1] = OP_STOP;
        break;
      }

      char c = *src++;
      switch (state) {
        case PathInit:
          if (c == 'B') {
            // actions[p++] = (OP_BEGIN);
            state = PathStart;
          } else {
            actions[p++] = (ACT_ERR_BEGIN);
            state = PathStop;
          }
          break;
        case PathStart:
          if (c == 'F') {
            runLength = 1;
            state = PathOrtho_F;
          } else if (c == 'R') {
            actions[p++] = (ACT_ERR_NOF);
            state = PathStop;
          } else if (c == 'L') {
            actions[p++] = (ACT_ERR_NOF);
            state = PathStop;
          } else if (c == 'X') {
            state = PathExit;
          } else if (c == 'S') {
            state = PathStop;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathOrtho_F:
          if (c == 'F') {
            runLength++;
          } else if (c == 'R') {
            actions[p++] = (FWD0 + runLength);
            state = PathOrtho_R;
          } else if (c == 'L') {
            actions[p++] = (FWD0 + runLength);
            state = PathOrtho_L;
          } else if (c == 'X') {
            actions[p++] = (FWD0 + runLength);
            state = PathExit;
          } else if (c == 'S') {
            actions[p++] = (FWD0 + runLength);
            state = PathStop;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathOrtho_R:
          if (c == 'F') {
            actions[p++] = (SS90ER);
            runLength = 2;
            state = PathOrtho_F;
          } else if (c == 'R') {
            actions[p++] = (SS90ER);
            actions[p++] = (FWD1);
            state = PathOrtho_R;
          } else if (c == 'L') {
            actions[p++] = (SS90ER);
            actions[p++] = (FWD1);
            state = PathOrtho_L;
          } else if (c == 'X') {
            actions[p++] = (SS90ER);
            actions[p++] = (FWD1);
            state = PathExit;
          } else if (c == 'S') {
            actions[p++] = (SS90ER);
            actions[p++] = (FWD1);
            state = PathStop;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathOrtho_L:
          if (c == 'F') {
            actions[p++] = (SS90EL);
            runLength = 2;
            state = PathOrtho_F;
          } else if (c == 'R') {
            actions[p++] = (SS90EL);
            actions[p++] = (FWD1);
            state = PathOrtho_R;
          } else if (c == 'L') {
            actions[p++] = (SS90EL);
            actions[p++] = (FWD1);
            state = PathOrtho_L;
          } else if (c == 'X') {
            actions[p++] = (SS90EL);
            actions[p++] = (FWD1);
            state = PathExit;
          } else if (c == 'S') {
            actions[p++] = (SS90EL);
            actions[p++] = (FWD1);
            state = PathStop;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathStop:
          actions[p] = (OP_STOP);  // make sure the action list gets terminated
          state = PathFinish;
          break;
        case PathExit:
          actions[p++] = (ACT_EXPLORE);
          actions[p] = (OP_STOP);  // make sure the action list gets terminated
          state = PathFinish;
          break;
        default:
          actions[p++] = (ACT_ERROR);
          state = PathFinish;
          break;
      }
    }
  }

  void makeDiagonalActions(const char *src, uint8_t *actions, const uint16_t maxLength) {
    int runLength = 0;
    int p = 0;
    pathgen_state_t state = PathInit;
    while (state != PathFinish) {
      if (runLength > 63) {  // MAGIC: maximum for hald-size maze
        actions[0] = ACT_ERROR;
        actions[1] = OP_STOP;
        break;
      }
      if (p >= maxLength) {
        actions[0] = ACT_ERROR;
        actions[1] = OP_STOP;
        break;
      }
      char c = *src++;
      switch (state) {
        case PathInit:
          if (c == 'B') {
            // actions[p++] = (OP_BEGIN);
            state = PathStart;
          } else {
            actions[p++] = (ACT_ERR_BEGIN);
            state = PathStop;
          }
          break;
        case PathStart:
          if (c == 'F') {
            runLength = 1;
            state = PathOrtho_F;
          } else if (c == 'R') {
            actions[p++] = (ACT_ERR_NOF);
            state = PathStop;
          } else if (c == 'L') {
            actions[p++] = (ACT_ERR_NOF);
            state = PathStop;
          } else if (c == 'S') {
            state = PathStop;
          } else if (c == 'X') {
            state = PathExit;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathOrtho_F:
          if (c == 'F') {
            runLength++;
          } else if (c == 'R') {
            actions[p++] = (FWD0 + runLength);
            state = PathOrtho_R;
          } else if (c == 'L') {
            actions[p++] = (FWD0 + runLength);
            state = PathOrtho_L;
          } else if (c == 'S') {
            actions[p++] = (FWD0 + runLength);
            state = PathStop;
          } else if (c == 'X') {
            actions[p++] = (FWD0 + runLength);
            state = PathExit;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathOrtho_R:
          if (c == 'F') {
            actions[p++] = (SS90FR);
            runLength = 2;
            state = PathOrtho_F;
          } else if (c == 'R') {
            state = PathOrtho_RR;
          } else if (c == 'L') {
            actions[p++] = (SD45R);
            runLength = 2;
            state = PathDiag_RL;
          } else if (c == 'S') {
            actions[p++] = (SS90ER);
            actions[p++] = (FWD1);
            state = PathStop;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathOrtho_L:
          if (c == 'F') {
            actions[p++] = (SS90FL);
            runLength = 2;
            state = PathOrtho_F;
          } else if (c == 'R') {
            actions[p++] = (SD45L);
            runLength = 2;
            state = PathDiag_LR;
          } else if (c == 'L') {
            state = PathOrtho_LL;
          } else if (c == 'S') {
            actions[p++] = (SS90EL);
            actions[p++] = (FWD1);
            state = PathStop;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathOrtho_RR:
          if (c == 'F') {
            actions[p++] = (SS180R);
            runLength = 2;
            state = PathOrtho_F;
          } else if (c == 'R') {
            actions[p++] = (ACT_ERR_RRR);
            state = PathStop;
          } else if (c == 'L') {
            actions[p++] = (SD135R);
            runLength = 2;
            state = PathDiag_RL;
          } else if (c == 'S') {
            actions[p++] = (SS180R);
            actions[p++] = (FWD1);
            state = PathStop;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathDiag_RL:
          if (c == 'F') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DS45L);
            runLength = 2;
            state = PathOrtho_F;
          } else if (c == 'R') {
            runLength += 1;
            state = PathDiag_LR;
          } else if (c == 'L') {
            state = PathDiag_LL;
          } else if (c == 'S') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DS45L);
            actions[p++] = (FWD1);
            state = PathStop;
          } else if (c == 'X') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DS45L);
            actions[p++] = (FWD1);
            state = PathExit;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathDiag_LR:
          if (c == 'F') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DS45R);
            runLength = 2;
            state = PathOrtho_F;
          } else if (c == 'R') {
            state = PathDiag_RR;
          } else if (c == 'L') {
            runLength += 1;
            state = PathDiag_RL;
          } else if (c == 'S') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DS45R);
            actions[p++] = (FWD1);
            state = PathStop;
          } else if (c == 'X') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DS45R);
            actions[p++] = (FWD1);
            state = PathExit;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathOrtho_LL:
          if (c == 'F') {
            actions[p++] = (SS180L);
            runLength = 2;
            state = PathOrtho_F;
          } else if (c == 'R') {
            actions[p++] = (SD135L);
            runLength = 2;
            state = PathDiag_LR;
          } else if (c == 'L') {
            actions[p++] = (ACT_ERR_LLL);
            state = PathStop;
          } else if (c == 'S') {
            actions[p++] = (SS180L);
            actions[p++] = (FWD1);
            state = PathStop;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathDiag_LL:
          if (c == 'F') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DS135L);
            runLength = 2;
            state = PathOrtho_F;
          } else if (c == 'R') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DD90L);
            runLength = 2;
            state = PathDiag_LR;
          } else if (c == 'L') {
            actions[p++] = (ACT_ERR_LLL);
            state = PathStop;
          } else if (c == 'S') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DS135L);
            actions[p++] = (FWD1);
            state = PathStop;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathDiag_RR:
          if (c == 'F') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DS135R);
            runLength = 2;
            state = PathOrtho_F;
          } else if (c == 'R') {
            state = PathDiag_RR;
          } else if (c == 'L') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DD90R);
            runLength = 2;
            state = PathDiag_RL;
          } else if (c == 'S') {
            actions[p++] = (DIA0 + runLength);
            actions[p++] = (DS135R);
            actions[p++] = (FWD1);
            state = PathStop;
          } else {
            actions[p++] = (ACT_ERR_END);
            state = PathStop;
          }
          break;
        case PathStop:
          actions[p++] = (OP_STOP);  // make sure the action list gets terminated
          state = PathFinish;
          break;
        case PathExit:
          actions[p++] = (ACT_EXPLORE);
          actions[p++] = (OP_STOP);  // make sure the action list gets terminated
          state = PathFinish;
          break;
        default:
          actions[p++] = (ACT_ERROR);
          state = PathFinish;
          break;
      }
    }
  }

 private:
};
