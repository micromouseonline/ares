//
// Created by peter on 04/02/25.
//

#pragma once
#include <stdint.h>
#include <iostream>

//////////////////////////////////////////////////////////////////////////////////
class Board {
 public:
  static Board& getInstance(void* params = nullptr) {
    static Board instance(params);  // Meyers Singleton
    return instance;
  }

  void init(void* params = nullptr) {
    (void)params;
    std::cout << "Board initialisation." << std::endl;
  }

  void beep() {
    std::cout << "      BEEP!" << std::endl;
  }

 private:
  Board(void* params = nullptr) {
    if (params) {
      std::cout << "Board Constructor with parameters" << std::endl;
    } else {
      std::cout << "Board Constructor without parameters" << std::endl;
    }
  }
};

/////////////////////////////////////////////////////////////////////////////////
class Vehicle {
 public:
  static Vehicle& getInstance() {
    static Vehicle instance(Board::getInstance());  // ✅ Correct Meyers Singleton
    return instance;
  }

  void move() {
    std::cout << "    Vehicle Moving" << std::endl;
    m_board.beep();
  }

 private:
  explicit Vehicle(Board& board)
      : m_board(board) {
    std::cout << "Vehicle Constructor." << std::endl;
  }

  Board& m_board;

  ~Vehicle() = default;
  Vehicle(const Vehicle&) = delete;
  Vehicle& operator=(const Vehicle&) = delete;
  Vehicle(Vehicle&&) = delete;
  Vehicle& operator=(Vehicle&&) = delete;
};

/////////////////////////////////////////////////////////////////////////////////

class Behaviour {
 public:
  static Behaviour& getInstance() {
    static Behaviour instance(Vehicle::getInstance());  // Meyers Singleton with DI
    return instance;
  }

  void run() {
    std::cout << "  Behaviour running." << std::endl;
    perform();
  }

  // Calculate the speed for the robot to move forward
  void perform() {
    std::cout << "  Behaviour is performing " << std::endl;
    m_vehicle.move();
  }

 private:
  explicit Behaviour(Vehicle& vehicle)
      : m_vehicle(vehicle) {
    std::cout << "  Behaviour constructor." << std::endl;
  }
  Vehicle& m_vehicle;
};

//////////////////////////////////////////////////////////////////////////////////

class Robot {
 public:
  static Robot& getInstance() {
    static Robot instance(Behaviour::getInstance());  // Meyers Singleton with DI
    return instance;
  }

  void run() {
    std::cout << "Robot running." << std::endl;
    m_behaviour.run();
  }

 private:
  explicit Robot(Behaviour& behaviour)
      : m_behaviour(behaviour) {
    std::cout << "Robot Constructor." << std::endl;
  }
  Behaviour& m_behaviour;
};
