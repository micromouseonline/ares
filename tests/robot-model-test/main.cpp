
#include <stdint.h>
#include <iostream>
#include "robot.h"
#include "stdio.h"
/////////////////////////////////////////////////////////////////////////////////

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
  BasicBoard& board = BasicBoard::getInstance();  // Singleton board;
  board.beep(100);
  board.setMotorVolts(1, 2);
  //  /// give that to the Vehicle and have the vehicle initialise itself
  Vehicle& vehicle = Vehicle::getInstance();
  vehicle.setSpeed(123);
  /// now give the vehicle to the behaviour and let it initialise itself
  Behaviour mouse = Behaviour::getInstance();
  /// Finally turn it loose
  mouse.run();
  return 0;
}
