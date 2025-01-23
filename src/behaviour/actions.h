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

#include <stdint.h>
#include "config.h"
#include "trajectory.h"
/***
 * TODO - there are 256 different possible actions. Many of these are not
 *        used yet all of those that are used should have a string name.
 *        This can be accomplished with an array of std::pair and a function
 *        that retrieves the name.
 *        OR SO YOU MAY THINK
 *        Be aware that the compiler, if set to O3, might store multiple
 *        copies of the strings in a misquided attempt to 'unroll' the access.
 *        At least, according to Godbolt:
 *        https://godbolt.org/z/fMTfq6xbT
 *
 *        If using O2 or O2, the compiler may not do this unrolling
 *        and this code may be good.

#include <array>
#include <cstdint>
#include <utility>
#include <cstddef>

constexpr std::array<std::pair<uint8_t, const char*>,6> valueStringMap = {{
    {1, "One"},
    {3, "Three"},
    {5, "Five"},
    {7, "Seven"},
    {9, "Nine"},
    {19, "Nineteen"},
}};

const char* lookupString(uint8_t value) {
  for (const auto& pair : valueStringMap) {
    if (pair.first == value) {
      return pair.second;
    }
  }
  return nullptr; // Return nullptr if not found
}

   If you do see the unrolling, just use an array and live with the large number
   of empty strings. It will be smaller and faster.

   I thnk that, on the STM32, O2 is the better optimisation setting
 */

#define NUM_ACTIONS_MAX 256

/*
 * mouse action bit values:
 *
 * Straights: 0-31 used
 *  00LLLLLL
 *  ||||||||
 *  ||``````-- Run Length in Cells (0-63)
 *  ||
 *  ``-------- 00 => straight (orthogonal)
 *
 * Since there should never be a zero length orthogonal straight
 * it is convenient to use the command value 0x00 as an end marker
 *
 * Diagonals: 64-127 used
 *  01LLLLLL
 *  ||||||||
 *  ||``````-- Run Length in Cells (0-63)
 *  ||
 *  ``-------- 01 => diagonal
 *
 * Turns use 6 bits in two groups of 32
 *
 * Turns - In Place 128-
 *  10000TTD
 *  ||||||||
 *  |||||||`--  0 => Turn Right
 *  |||||||     1 => Turn Left
 *  |||||||
 *  |||||``--- 00 => IP45  Turn
 *  |||||      01 => IP90  Turn
 *  |||||      10 => IP135 Turn
 *  |||||      11 => IP180 Turn
 *  |||||
 *  |||||
 *  `````----- 1000 => Turn In Place
 *
 *
 * Turns - Smooth (160-191)- there is scope for 16 different turns
 *  101TTTTD
 *  ||||||||
 *  |||||||`--  0 => Turn Right
 *  |||||||     1 => Turn Left
 *  |||||||
 *  |||````--- 0000 => SS90  Turn
 *  |||        0001 => SS180 Turn
 *  |||        0010 => SD45  Turn
 *  |||        0011 => SD135 Turn
 *  |||        0100 => DS45  Turn
 *  |||        0101 => DS135 Turn
 *  |||        0110 => DD90  Turn
 *  |||        0111 => SS90E Turn
 *  |||        1xxx => unused (available for complex turns)
 *  |||
 *  ```------- 101 => Turn Smooth
 *
 *
 * Messages/Flags:
 *  11MMMMMM
 *  ||||||||
 *  ||``````-- Message Code
 *  ||         000000 => Start
 *  ||         000001 => Stop
 *  ||         000010 => Explore
 *  ||         000011 => End
 *  ||         000100 => Handstart
 *  ||         01xxxx => unused
 *  ||         10xxxx => unused
 *  ||         11xxxx => Error Codes
 *  ||
 *  ``-------- 11 => Message
 *
 *
 * Binary constants are a C++14 feature. If you have a cross compiler for
 * a microcontroller it probably supports the binary constant syntax.
 * If not, sorry but you will have to convert the values by hand.
 */

// clang-format off
// clang-format on

// clang-format off

const uint8_t  OP_MASK_OP_TYPE     =  (0b11000000);
const uint8_t  OP_MASK_SQUARES     =  (0b00111111);
const uint8_t  OP_MASK_TURN_TYPE   =  (0b11100000);
const uint8_t  OP_MASK_TURN_INDEX  =  (0b00011111);
const uint8_t  OP_MASK_TURN_DIR    =  (0b00000001);
const uint8_t  OP_MASK_MSG_TYPE    =  (0b11110000);
const uint8_t  OP_MASK_MSG_INDEX   =  (0b00001111);



const uint8_t  OP_DIR_LEFT      =   1;
const uint8_t  OP_TYPE_ORTHO    =   0; // 0b00000000
const uint8_t  OP_TYPE_DIAG     =  64; // 0b01000000
const uint8_t  OP_TYPE_TURN     = 128; // 0b10000000
const uint8_t  OP_TYPE_MSG      = 192; // 0b11000000
const uint8_t  OP_MSG_CMD       = 192; // 0b11000000
const uint8_t  OP_MSG_ERR       = 240; // 0b11110000

const uint8_t  OP_TURN_INPLACE  = 128; // 0b10000000
const uint8_t  OP_TURN_SMOOTH   = 160; // 0b10100000

const uint8_t  OP_STOP       =0x00;

enum : uint8_t{
  FWD0 = OP_TYPE_ORTHO,
  FWD1,   FWD2,   FWD3,   FWD4,   FWD5,   FWD6,   FWD7,
  FWD8,   FWD9,   FWD10,  FWD11,  FWD12,  FWD13,  FWD14,  FWD15,
  FWD16,  FWD17,  FWD18,  FWD19,  FWD20,  FWD21,  FWD22,  FWD23,
  FWD24,  FWD25,  FWD26,  FWD27,  FWD28,  FWD29,  FWD30,  FWD31,

  DIA0 = OP_TYPE_DIAG,
  DIA1,   DIA2,   DIA3,   DIA4,   DIA5,   DIA6,   DIA7,
  DIA8,   DIA9,   DIA10,  DIA11,  DIA12,  DIA13,  DIA14,  DIA15,
  DIA16,  DIA17,  DIA18,  DIA19,  DIA20,  DIA21,  DIA22,  DIA23,
  DIA24,  DIA25,  DIA26,  DIA27,  DIA28,  DIA29,  DIA30,  DIA31,
  DIA32,  DIA33,  DIA34,  DIA35,  DIA36,  DIA37,  DIA38,  DIA39,
  DIA40,  DIA41,  DIA42,  DIA43,  DIA44,  DIA45,  DIA46,  DIA47,
  DIA48,  DIA49,  DIA50,  DIA51,  DIA52,  DIA53,  DIA54,  DIA55,
  DIA56,  DIA57,  DIA58,  DIA59,  DIA60,  DIA61,  DIA62,  DIA63,

  IP45R = OP_TURN_INPLACE,   IP45L,
  IP90R,                     IP90L,
  IP135R,                    IP135L,
  IP180R,                    IP180L,
  IP_END,

  SS90SR = OP_TURN_SMOOTH,   SS90SL,
  SS90FR,                    SS90FL,
  SS180R,                    SS180L,
  SD45R,                     SD45L,
  SD135R,                    SD135L,
  DS45R,                     DS45L,
  DS135R,                    DS135L,
  DD90R,                     DD90L,
  SS90ER,                    SS90EL,
  SMOOTH_END,

  ACT_BEGIN    =  192,
  ACT_BEGIN_HS =  193,
  ACT_EXPLORE  =  194,
  ACT_END      =  0,
  ACT_NONE = 0,


  ACT_ERR_BASE  = 240,
  ACT_ERR_NOF   = 240,
  ACT_ERR_RRR   = 241,
  ACT_ERR_LLL   = 242,
  ACT_ERR_BEGIN = 243,
  ACT_ERR_END   = 244,
  ACT_ERROR     = 255,
};

constexpr const char* actionNames[256] = {
    /* 0x00 - 0x3F: Orthogonal Moves */
    "END   ", "FWD1  ", "FWD2  ", "FWD3  ", "FWD4  ", "FWD5  ", "FWD6  ", "FWD7  ",
    "FWD8  ", "FWD9  ", "FWD10 ", "FWD11 ", "FWD12 ", "FWD13 ", "FWD14 ", "FWD15 ",
    "FWD16 ", "FWD17 ", "FWD18 ", "FWD19 ", "FWD20 ", "FWD21 ", "FWD22 ", "FWD23 ",
    "FWD24 ", "FWD25 ", "FWD26 ", "FWD27 ", "FWD28 ", "FWD29 ", "FWD30 ", "FWD31 ",
    "------", "------", "------", "------", "------", "------", "------", "------",
    "------", "------", "------", "------", "------", "------", "------", "------",
    "------", "------", "------", "------", "------", "------", "------", "------",
    "------", "------", "------", "------", "------", "------", "------", "------",
    /// TODO: and what aboud DIA0
    /* 0x40 - 0x7F: Diagonal Moves */
    "DIA0  ", "DIA1  ", "DIA2  ", "DIA3  ", "DIA4  ", "DIA5  ", "DIA6  ", "DIA7  ",
    "DIA8  ", "DIA9  ", "DIA10 ", "DIA11 ", "DIA12 ", "DIA13 ", "DIA14 ", "DIA15 ",
    "DIA16 ", "DIA17 ", "DIA18 ", "DIA19 ", "DIA20 ", "DIA21 ", "DIA22 ", "DIA23 ",
    "DIA24 ", "DIA25 ", "DIA26 ", "DIA27 ", "DIA28 ", "DIA29 ", "DIA30 ", "DIA31 ",
    "DIA32 ", "DIA33 ", "DIA34 ", "DIA35 ", "DIA36 ", "DIA37 ", "DIA38 ", "DIA39 ",
    "DIA40 ", "DIA41 ", "DIA42 ", "DIA43 ", "DIA44 ", "DIA45 ", "DIA46 ", "DIA47 ",
    "DIA48 ", "DIA49 ", "DIA50 ", "DIA51 ", "DIA52 ", "DIA53 ", "DIA54 ", "DIA55 ",
    "DIA56 ", "DIA57 ", "DIA58 ", "DIA59 ", "DIA60 ", "DIA61 ", "DIA62 ", "DIA63 ",
    /* 0x80 - 0x9F: In-Place Turns */
    "IP45R ", "IP45L ", "IP90R ", "IP90L ", "IP135R", "IP135L", "IP180R", "IP180L",
    "------", "------", "------", "------", "------", "------", "------", "------",
    "------", "------", "------", "------", "------", "------", "------", "------",
    "------", "------", "------", "------", "------", "------", "------", "------",
    /* 0xA0 - 0xBF: Smooth Turns */
    "SS90SR", "SS90SL", "SS90FR", "SS90FL", "SS180R", "SS180L", "SD45R ", "SD45L ",
    "SD135R", "SD135L", "DS45R ", "DS45L ", "DS135R", "DS135L", "DD90R ", "DD90L ",
    "SS90ER", "SS90EL", "------", "------", "------", "------", "------", "------",
    "------", "------", "------", "------", "------", "------", "------", "------",
    /* 0xC0 - 0xFF: Messages */
    "BEG_FS", "BEG_HS", "EXPLOR", "END   ", "------", "------", "------", "------",
    "------", "------", "------", "------", "------", "------", "------", "------",
    "ERR00 ", "ERR01 ", "ERR02 ", "ERR03 ", "ERR04 ", "ERR05 ", "ERR06 ", "ERR07 ",
    "ERR08 ", "ERR09 ", "ERR10 ", "ERR11 ", "ERR12 ", "ERR13 ", "ERR14 ", "ERR15 ",
    "ERR16 ", "ERR17 ", "ERR18 ", "ERR19 ", "ERR20 ", "ERR21 ", "ERR22 ", "ERR23 ",
    "ERR24 ", "ERR25 ", "ERR26 ", "ERR27 ", "ERR28 ", "ERR29 ", "ERR30 ", "ERR31 ",
    "------", "------", "------", "------", "------", "------", "------", "------",
    "------", "------", "------", "------", "------", "------", "------", "------",
};

inline const char* getActionName(uint8_t op_code) {
    return actionNames[op_code];
}


const bool is_ortho_in[]  = { true,  true,  true,  true,  true,  true,  true,  true,  true,  true, false, false, false, false, false, false, true, true};
const bool is_ortho_out[] = { true,  true,  true,  true,  true,  true, false, false, false, false,  true,  true,  true,  true, false, false, true, true};
// clang-format on

struct Action {
  uint8_t op_code;

  explicit Action(int c = OP_STOP)
      : op_code(c) {
  }

  uint8_t direction() const {
    return op_code & OP_MASK_TURN_DIR;
  }
  uint8_t length() const {
    return op_code & OP_MASK_SQUARES;
  }

  const char* name() {
    return actionNames[op_code];
  }

  int get_smooth_turn_type() const {
    return op_code - OP_TURN_SMOOTH;
  }

  int get_spin_turn_type() const {
    return op_code - OP_TURN_INPLACE;
  }

  bool is_left_turn() const {
    return direction() == OP_DIR_LEFT;
  }

  bool is_ortho_straight() const {
    return ((op_code >= FWD0) && (op_code <= FWD31));
  }

  bool is_diagonal_straight() const {
    return ((op_code >= DIA0) && (op_code <= DIA63));
  }

  bool is_straight_move() const {
    return is_ortho_straight() || is_diagonal_straight();
  }

  bool is_ortho_entry() {
    if (is_ortho_straight()) {
      return true;
    }
    if (is_diagonal_straight()) {
      return false;
    }
    if (is_spin_turn()) {
      return true;
    }
    if (is_smooth_turn()) {
      int index = get_smooth_turn_type();
      return is_ortho_in[index];
    }
    return false;
  }

  bool is_ortho_exit() {
    if (is_ortho_straight()) {
      return true;
    }
    if (is_diagonal_straight()) {
      return false;
    }
    if (is_spin_turn()) {
      return true;
    }
    if (is_smooth_turn()) {
      int index = get_smooth_turn_type();
      return is_ortho_out[index];
    }
    return false;
  }

  bool is_turn_move() const {
    return (is_spin_turn() || is_smooth_turn());
  }

  bool is_smooth_turn() const {
    return (op_code >= OP_TURN_SMOOTH && op_code < SMOOTH_END);
  }

  bool is_spin_turn() const {
    return (op_code >= OP_TURN_INPLACE && op_code < IP_END);
  }

  float spinTurnAngle() {
    return 45.0f * (1 + (get_spin_turn_type() / 2));
  }

  bool operator==(const uint8_t v) const {
    return op_code == v;
  }
  bool operator<=(const uint8_t v) const {
    return op_code <= v;
  }
  bool operator>=(const uint8_t v) const {
    return op_code >= v;
  }
  bool operator!=(const uint8_t v) const {
    return op_code != v;
  }
};
