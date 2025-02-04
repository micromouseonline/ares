
#include <stdint.h>
#include <iostream>
#include "classes.h"
#include "stdio.h"
/////////////////////////////////////////////////////////////////////////////////
/***
 * Simple program to model the assembly of parts of the mouse
 */

int main() {
  int id = 99;
  std::cout << "\nLet's create a couple of boards..." << std::endl;
  IBoard* boop_board = &BoopBoard::getInstance(&id);  // ;
  IBoard* beep_board = &BeepBoard::getInstance(&id);  // ;

  std::cout << "\n... and a vehicle" << std::endl;
  IVehicle* vehicle = &Vehicle::getInstance();

  std::cout << "\n... and a behaviour" << std::endl;
  IBehaviour* behaviour = &Behaviour::getInstance();

  std::cout << "\n... and use them to make a robot" << std::endl;
  Robot robot = RobotFactory::createRobot(boop_board, vehicle, behaviour);
  robot.run();
  return 0;
}
