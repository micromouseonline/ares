//
// Created by peter on 03/02/25.
//

#pragma once
#include <stdint.h>
#include <iostream>
#include "vehicle/hal/board-interface.h"
/***
 * Simple program to model the assembly of parts of the mouse
 */

/////////////////////////////////////////////////////////////////////////////////
class Vehicle {
 public:
  static Vehicle& getInstance(BoardInterface* board = nullptr) {
    /// If no pointer to a board is provided, use a BasicBoard
    static Vehicle instance(board ? *board : BasicBoard::getInstance());  // ✅ Correct Meyers Singleton
    return instance;
  }

  void setSpeed(int speed) {
    std::cout << "Vehicle: Speed set to " << speed << std::endl;
    float volts = speed * 3.0f;
    m_board.setMotorVoltage(volts, volts);
  }

 private:
  explicit Vehicle(BoardInterface& board)
      : m_board(board) {
    std::cout << "Vehicle initialized." << std::endl;
  }

  ~Vehicle() = default;

  // Delete copy/move constructors and assignment operators
  Vehicle(const Vehicle&) = delete;
  Vehicle& operator=(const Vehicle&) = delete;
  Vehicle(Vehicle&&) = delete;
  Vehicle& operator=(Vehicle&&) = delete;

  BoardInterface& m_board;
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
    std::cout << "Behaviour initialized." << std::endl;
  }
  int calculateSpeed(int distance) {
    return distance * 17;  // Example speed value
  }

  Vehicle& m_vehicle;
};
