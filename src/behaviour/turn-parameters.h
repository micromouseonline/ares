//
// Created by peter on 14/02/24.
//

#pragma once
#include "stdio.h"

struct CubicTurnParameters {
  float speed_max;
  float in_offset;    // (mm) amount to finish early to catch edge
  float turn_offset;  // (mm) distance after edge to do turn
  float out_offset;   // mm distance from centre after turn finishes
  float angle;        // deg
  float length;

  void print() const {
    printf(" %5.0f %5.0f %5.0f %4.1f %5.1f\n", speed_max, in_offset, out_offset, angle, length);
  }
};

// clang-format off

/// SPEED ADJUST ???
/// FOR 10m/s/s subtract 5mm from turn offset
/// FOR 12m/s/s subtract 10mm from turn offset

/// SS90E should trigger at offset = 100?

/// All set up at 5m/s/s
// revised tighter turns to ensure the posts get seen
const CubicTurnParameters cubic_params[] = {
//   max speed     in turn, out   angle   len
    {     795,    200,  90, 100,  -90.0,  149 },  //  SS90SR(not used)[ ]/ check APEC 24
    {     793,    200,  90, 100,   91.0,  150 },  //  SS90SL(not used)[ ]/ check APEC 24
    {    1000,    210,  75, 120,  -90.5,  195 },  //  SS90FR         /[x]/ check APEC 24
    {    1000,    210,  70, 115,   90.0,  195 },  //  SS90FL         /[x]/ check APEC 24
    {    1000,    210,  80, 110, -181.0,  365 },  //  SS180FR        /[x]/ check APEC 24
    {    1000,    210,  80, 110,  180.0,  365 },  //  SS180FL        /[x]/ check APEC 24
    {     750,    200,  45,  75,  -45.0,  100 },  //  SD45R          /[x]/ check APEC 24
    {     850,    200,  45,  75,   45.5,  100 },  //  SD45L          /[x]/ check APEC 24
    {     800,    200,  73,  85, -135.5,  230 },  //  SD135R         /[x]/ check APEC 24
    {     700,    200,  73,  70,  135.5,  230 },  //  SD135L         /[x]/ check APEC 24
    {     850,    165,  90, 175,  -45.2,  100 },  //  DS45R          /[x]/ check APEC 24 NOTE: this is a potential problem if followed by SS90F
    {     850,    165,  85, 175,   45.5,  100 },  //  DS45L          /[x]/ check APEC 24
    {     650,    165,  65, 130, -135.0,  220 },  //  DS135R         /[x]/ check APEC 24
    {     650,    165,  70, 130,  134.5,  220 },  //  DS135L         /[x]/ check APEC 24
    {     600,    165,  65,  85,  -90.0,  125 },  //  DD90R          /[x]/ check APEC 24
    {     600,    165,  63,  85,   90.2,  125 },  //  DD90L          /[x]/ check APEC 24
    {     500,    210, 120,  70,  -90.6,  116 },  //  SS90ER         /[x]/ check APEC 24
    {     500,    210, 120,  65,   90.0,  116 },  //  SS90EL         /[x]/ check APEC 24
};

const CubicTurnParameters old_cubic_params[] = {
//   max speed     in turn, out   angle   len
    {     795,    200,  90, 100,  -90.0,  149 },  //  SS90SR(not used)[ ]/ check APEC 24
    {     793,    200,  90, 100,   91.0,  150 },  //  SS90SL(not used)[ ]/ check APEC 24
    {    1100,    210,  50, 131,  -90.0,  220 },  //  SS90FR         /[x]/ check APEC 24
    {    1100,    210,  55, 131,   90.0,  220 },  //  SS90FL         /[x]/ check APEC 24
    {    1000,    210,  60, 120, -180.0,  360 },  //  SS180FR        /[x]/ check APEC 24
    {    1000,    210,  60, 120,  180.0,  380 },  //  SS180FL        /[x]/ check APEC 24
    {    1070,    200,  20,  70,  -45.0,  135 },  //  SD45R          /[x]/ check APEC 24
    {    1064,    200,  20,  55,   45.0,  135 },  //  SD45L          /[x]/ check APEC 24
    {     857,    200,  25, 100, -135.0,  260 },  //  SD135R         /[x]/ check APEC 24
    {     856,    200,  35,  90,  135.0,  260 },  //  SD135L         /[x]/ check APEC 24
    {    1070,    150,  70, 155,  -45.0,  120 },  //  DS45R          /[x]/ check APEC 24
    {    1064,    150,  70, 155,   44.0,  120 },  //  DS45L          /[x]/ check APEC 24
    {     857,    150,  20, 140, -135.0,  260 },  //  DS135R         /[x]/ check APEC 24
    {     856,    150,  35, 130,  135.0,  260 },  //  DS135L         /[x]/ check APEC 24
    {     861,    150,  25, 100,  -90.0,  175 },  //  DD90R          /[x]/ check APEC 24
    {     854,    150,  28,  90,   90.0,  175 },  //  DD90L          /[x]/ check APEC 24
    {     703,    210, 115,  60,  -90.0,  116 },  //  SS90ER         /[x]/ check APEC 24
    {     690,    210, 115,  60,   90.0,  116 },  //  SS90EL         /[x]/ check APEC 24
};
// clang-format on

//

#endif  // MAZERUNNER32_TURN_PARAMETERS_H
