
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
class Board {
 public:
  Board() {
    initialize();
  }
  void initialize() {
    // Initialize hardware peripherals
    std::cout << "Board initialized." << std::endl;
  }

  // example method
  void setMotorVolts(float left, float right) {
    std::cout << "Board:  Left Volts: " << left << ", Right Volts: " << right << std::endl;
  }

 private:
  // Add member variables for hardware peripherals
};

/////////////////////////////////////////////////////////////////////////////////

class Vehicle {
 public:
  Vehicle(Board& board)
      : m_board(board) {
    std::cout << "Vehicle initialized." << std::endl;
  }

  void setSpeed(int speed) {
    // Calculate voltages for left and right motors
    int leftVoltage = speed * 3.0f;   // Simplified calculation for example
    int rightVoltage = speed * 3.0f;  // Simplified calculation for example

    std::cout << "Vehicle: calculate voltage for speed: " << speed << std::endl;
    m_board.setMotorVolts(leftVoltage, rightVoltage);
  }

 private:
  // Delete copy constructor and assignment operator
  Vehicle(const Vehicle&) = delete;
  Vehicle& operator=(const Vehicle&) = delete;
  Vehicle(Vehicle&&) = delete;
  Vehicle& operator=(Vehicle&&) = delete;
  Board& m_board;
  // Optionally add other members and methods
};

/////////////////////////////////////////////////////////////////////////////////

class Behaviour {
 public:
  Behaviour(Vehicle& vehicle)
      : m_vehicle(vehicle) {
    std::cout << "Behaviour initialized." << std::endl;
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
  int calculateSpeed(int distance) {
    return distance * 17;  // Example speed value
  }

  Vehicle& m_vehicle;
};

/////////////////////////////////////////////////////////////////////////////////

int main() {
  /// first createthe board and have it initialise itself
  Board board;
  /// give that to the Vehicle and have the vehicle initialise itself
  Vehicle vehicle(board);
  /// now give the vehicle to the behaviour and let it initialise itself
  Behaviour mouse(vehicle);
  /// Finally turn it loose
  mouse.run();
  return 0;
}
