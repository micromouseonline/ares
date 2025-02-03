
#include <stdint.h>
#include <iostream>
#include "stdio.h"

/***
 * Simple program to model the assembly of parts of the mouse
 */

/////////////////////////////////////////////////////////////////////////////////
/***
 * the board is specific to an implementation. It has abstract drivers for any
 * anticipated hardware.
 *
 * Probably, there is a need for an abstract base class that defines the standard
 * peripherals. Specialised descendants can add or modify those as needed. It
 * would be possible for descendants to have direct hardware dependencies buried
 * inside them but it would be better to have those elsewhere so that they can be
 * shared between boards.
 */
#include <iostream>

class Board {
 public:
  static Board& getInstance() {
    static Board instance;  // Meyers Singleton
    return instance;
  }

  void setMotorVolts(float left, float right) {
    std::cout << "Board: Left Volts: " << left << ", Right Volts: " << right << std::endl;
  }

 private:
  Board() {
    std::cout << "\nBoard initialized." << std::endl;
  }

  ~Board() = default;

  // Delete copy/move constructors and assignment operators
  Board(const Board&) = delete;
  Board& operator=(const Board&) = delete;
  Board(Board&&) = delete;
  Board& operator=(Board&&) = delete;
};

/////////////////////////////////////////////////////////////////////////////////
class Vehicle {
 public:
  static Vehicle& getInstance() {
    static Vehicle instance(Board::getInstance());  // Meyers Singleton with DI
    return instance;
  }

  void setSpeed(int speed) {
    std::cout << "Vehicle: Speed set to " << speed << std::endl;
    float volts = speed * 3.0f;
    m_board.setMotorVolts(volts, volts);
  }

 private:
  explicit Vehicle(Board& board)
      : m_board(board) {
    std::cout << "\nVehicle initialized." << std::endl;
  }

  ~Vehicle() = default;

  // Delete copy/move constructors and assignment operators
  Vehicle(const Vehicle&) = delete;
  Vehicle& operator=(const Vehicle&) = delete;
  Vehicle(Vehicle&&) = delete;
  Vehicle& operator=(Vehicle&&) = delete;

  Board& m_board;
};

/////////////////////////////////////////////////////////////////////////////////

class Behaviour {
 public:
  static Behaviour& getInstance() {
    static Behaviour instance(Vehicle::getInstance());  // Meyers Singleton with DI
    return instance;
  }

  void run() {
    std::cout << "Behaviour running." << std::endl;
    move(100);
  }

  // Calculate the speed for the robot to move forward
  void move(int distance) {
    int speed = calculateSpeed(distance);
    std::cout << "Behaviour: Move for distance " << distance << " at speed: " << speed << std::endl;
    m_vehicle.setSpeed(speed);
  }

 private:
  explicit Behaviour(Vehicle& vehicle)
      : m_vehicle(vehicle) {
    std::cout << "\nBehaviour initialized." << std::endl;
  }
  int calculateSpeed(int distance) {
    return distance * 17;  // Example speed value
  }

  Vehicle& m_vehicle;
};

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
  Board& board = Board::getInstance();  // Singleton board;
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
