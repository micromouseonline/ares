//
// Created by peter on 09/01/25.
//

#pragma once

#include "maze.h"

/***
 * The PathFinder class will examine the data from a maze and generate
 * a string containg a path for the mouse to follow
 *
 * Assumes the maze is already flooded to a single target cell and so
 * every cell will have a cost that decreases as the target is approached.
 *
 * Starting at the given cell, the algorithm repeatedly looks for the
 * smallest available neighbour and records the action taken to reach it.
 *
 * The process starts by assuming the mouse is heading NORTH in the start
 * cell since that is what would be the case at the start of a speed run.
 *
 * At each cell, the preference is to move forwards if possible
 *
 * If the pathfinder is called from any other cell, the mouse must first
 * turn to face to the smallest neighbour of that cell using the same
 * method as in this function.
 *
 * The resulting path is a simple string, null terminated, that can be
 * printed to the Serial.to make it easy to compare paths using different
 * flooding or path generating methods.
 *
 * The characters in the path string are:
 * 	'B' : always the first character, it marks the path start.
 * 	'F' : move forwards a full cell
 * 	'H' : used in speedruns to indicate movement of half a cell forwards
 * 	'R' : turn right in this cell
 * 	'L' : turn left in this cell
 * 	'A' : turn around (should never happen in a speedrun path)
 * 	'S' : the last character in the path, telling the mouse to stop
 *
 * For example, the Japan2007 maze, flooded with a simple Manhattan
 * flood, might produce the path string:
 *
 * FFFRLLRRLLRRLLRFFRRFLLFFLRFRRLLRRLLRFFFFFFFFFRFFFFFRLRLLRRLLRRFFRFFFLFFFS
 *
 * The path string is processed by the mouse directly to make it move
 * along the path. At its simplest, this is just a case of executing
 * a single movement for each character in the string, using in-place turns.
 *
 * I would strongly recommend this style of path string. Not only can the
 * strings be used to compare routes very easily, they can be printed and
 * visually compared or followed by hand.
 *
 * Path strings are easily translated into more complex paths using
 * smooth turns an they are relatively easy to turn into a set of
 * commands that will represent a diagonal path.
 *
 * Further, short path strings can be hand-generated to test the movement
 * of the mouse or to test the setup of different turn types.
 *
 */

char pathOptions[16] = {'F', 'S', 'R', 'S', 'A', 'S', 'L', 'S'};
class PathFinder {
  /***
   * TODO: there is no protectiona against overrunning the path string
   * @param maze         a reference to the maze
   * @param path_string  pointer to a buffer of characters to hold the path
   * @param start        location of the first cell on the path
   * @param target       location of the target cell
   * @return             heading the mouse will have when stopped at the target
   */
  static Direction path_make_string(Maze& maze, char* path_string, Location start, Location target) {
    int cellCount = 0;
    char* pPath = path_string;
    *pPath++ = 'B';
    char command = 'F';
    Direction headingLast = maze.direction_to_smallest(start, DIR_N);
    Location here = start.neighbour(headingLast);
    Direction headingHere = maze.direction_to_smallest(here, DIR_N);
    Direction mEndHeading = headingLast;
    *pPath++ = command;
    while (here != target) {
      unsigned char hdgChange = (DIR_COUNT + headingHere - headingLast) % DIR_COUNT;
      command = pathOptions[hdgChange];
      *pPath++ = command;
      cellCount++;
      if (command == 'R') {
        mEndHeading = right_from(mEndHeading);
      }
      if (command == 'L') {
        mEndHeading = left_from(mEndHeading);
      }
      headingLast = headingHere;
      here = here.neighbour(headingHere);
      headingHere = maze.direction_to_smallest(here, headingLast);
    }
    *pPath++ = 'S';
    *pPath = 0;  // terminate with a null to make is c-string compatible
    return mEndHeading;
  }
};
