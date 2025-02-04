
#include <stdint.h>
#include <iostream>
#include "classes.h"
#include "stdio.h"
/////////////////////////////////////////////////////////////////////////////////
/***
 * Simple program to model the assembly of parts of the mouse
 */

int main() {
  /***
   * The Vehicle instantiates and initialises the board
   * The Mouse instantiates and initialises the vehicle
   *
   * Consequently we only need instantiate the Mose to get all the others done
   * in the right order.
   *
   * However, by doing them manually, in order, we could run intialisation,
   * configuration and test code before moving on
   *
   */
  //  /// first create the board and have it initialise itself
  int id = 99;
  Board& board = Board::getInstance(&id);  // Singleton board;
  //
  //  /// pointers are fine too
  //  //  Board* pBoard = &Board::getInstance();
  //
  //  //  /// give that to the Vehicle and have the vehicle initialise itself
  //  Vehicle& vehicle = Vehicle::getInstance(&board);
  //  /// now give the vehicle to the behaviour and let it initialise itself
  //  Behaviour mouse = Behaviour::getInstance();
  //  /// Finally turn it loose
  //  mouse.run();

  std::cout << "\nCreate the Robot" << std::endl;
  Robot robot = Robot::getInstance();
  std::cout << "\nRun The Robot" << std::endl;
  robot.run();

  return 0;
}
