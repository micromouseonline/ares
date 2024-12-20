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

#ifndef CONFIG_H
#define CONFIG_H

#define CODE "MAZERUNNER32"
/***
 * It is possible that you might want to run the robot in a number of
 * different mazes with different calibration values. The config file
 * can have different sensor defaults for each of these environments
 * so here you can define which set will be used.
 */

#define EVENT_HOME 1
#define EVENT_UK 2
#define EVENT_PORTUGAL 3
#define EVENT_APEC 4

#define EVENT EVENT_HOME
/***
 * The config.h file determines the actual robot variant that
 * will be the target for the build.
 *
 * This files lets you pick a specific robot that has its unique
 * configuration stored in that file. In this way, you can write
 * generic code that will work with a variety of different actual
 * robots. There are a number of example robot files in the project.
 * You should pick one that is closest to your setup, copy it and
 * then amend the details in that copy. Finally, add or modify
 * the selection settings below to use your new robot configuration.
 *
 */

/*************************************************************************/

/*************************************************************************/
/***
 * You may use a slightly different hardware platform than UKMARSBOT
 * Here you can include a suitable hardware configuration to define
 * things like IO pins, ADC channels and so on
 */

#define HARDWARE_UNKNOWN 0
#define HARDWARE_UKMARSBOT_1_3A 1
#define HARDWARE_DECIMUS4 2

#define HARDWARE HARDWARE_DECIMUS4

#if HARDWARE == HARDWARE_DECIMUS4
#include "chassis.h"
#include "pins.h"
#include "settings.h"
#else
#error "NO HARDWARE DEFINED"
#endif

/*************************************************************************/

#if EVENT == EVENT_HOME
#define GOAL Location(2, 5)
#else
#define GOAL Location(7, 7)
#endif

// This is the size, in mm,  for each cell in the maze.
const float FULL_CELL = 180.0f;
const float HALF_CELL = FULL_CELL * 0.5f;
const float DIAG_CELL = FULL_CELL * 0.707f;

#endif
