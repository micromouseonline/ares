//
// Created by peter on 04/02/25.
//

#pragma once
#include <stdint.h>
#include <iostream>

//////////////////////////////////////////////////////////////////////////////////
class IBoard {
 public:
  virtual void beep() = 0;
};

//////////////////////////////////////////////////////////////////////////////////
class BeepBoard : public IBoard {
 public:
  static BeepBoard& getInstance(void* params = nullptr) {
    static BeepBoard instance(params);  // Meyers Singleton
    return instance;
  }

  void init(void* params = nullptr) {
    (void)params;
    std::cout << "BeepBoard initialisation." << std::endl;
  }

  void beep() override {
    std::cout << "      BEEP!" << std::endl;
  }

 private:
  BeepBoard(void* params = nullptr) {
    if (params) {
      std::cout << "BeepBoard Constructor with parameters" << std::endl;
    } else {
      std::cout << "BeepBoard Constructor without parameters" << std::endl;
    }
  }
};

//////////////////////////////////////////////////////////////////////////////////
class BoopBoard : public IBoard {
 public:
  static BoopBoard& getInstance(void* params = nullptr) {
    static BoopBoard instance(params);  // Meyers Singleton
    return instance;
  }

  void init(void* params = nullptr) {
    (void)params;
    std::cout << "BoopBoard initialisation." << std::endl;
  }

  void beep() override {
    std::cout << "      BOOP!" << std::endl;
  }

 private:
  BoopBoard(void* params = nullptr) {
    if (params) {
      std::cout << "BoopBoard Constructor with parameters" << std::endl;
    } else {
      std::cout << "BoopBoard Constructor without parameters" << std::endl;
    }
  }
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
class IVehicle {
 public:
  virtual void init() = 0;
  virtual void move() = 0;
  virtual ~IVehicle() = default;
};

/////////////////////////////////////////////////////////////////////////////////
class Vehicle : public IVehicle {
 public:
  static Vehicle& getInstance(IBoard* board = nullptr) {
    static Vehicle instance(board ? *board : BeepBoard::getInstance());  // ✅ Correct Meyers Singleton
    return instance;
  }

  void init() override {
  }

  void setBoard(IBoard* board) {
    if (board) {
      m_board = board;
    }
  }

  void move() {
    std::cout << "    Vehicle Moving" << std::endl;
    m_board->beep();
  }

 private:
  explicit Vehicle(IBoard& board)
      : m_board(&board) {
    std::cout << "Vehicle Constructor." << std::endl;
  }

  IBoard* m_board;

  ~Vehicle() = default;
  Vehicle(const Vehicle&) = delete;
  Vehicle& operator=(const Vehicle&) = delete;
  Vehicle(Vehicle&&) = delete;
  Vehicle& operator=(Vehicle&&) = delete;
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
class IBehaviour {
 public:
  virtual void init() = 0;
  virtual void run() = 0;
  virtual ~IBehaviour() = default;
};

/////////////////////////////////////////////////////////////////////////////////

class Behaviour : public IBehaviour {
 public:
  static Behaviour& getInstance(IVehicle* vehicle = nullptr) {
    static Behaviour instance(vehicle ? *vehicle : Vehicle::getInstance());  // Meyers Singleton with DI
    return instance;
  }

  void init() override {
  }

  void setVehicle(IVehicle* vehicle) {
    if (vehicle) {
      m_vehicle = vehicle;
    }
  }

  void run() override {
    std::cout << "  Behaviour running." << std::endl;
    perform();
  }

  // Calculate the speed for the robot to move forward
  void perform() {
    std::cout << "  Behaviour is performing " << std::endl;
    m_vehicle->move();
  }

 private:
  explicit Behaviour(IVehicle& vehicle)
      : m_vehicle(&vehicle) {
    std::cout << "  Behaviour constructor." << std::endl;
  }
  IVehicle* m_vehicle;
};

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
class IRobot {
 public:
  virtual void run() = 0;
};
//////////////////////////////////////////////////////////////////////////////////

class Robot : public IRobot {
 public:
  static Robot& getInstance(IBehaviour* behaviour = nullptr) {
    static Robot instance(behaviour ? *behaviour : Behaviour::getInstance());  // Meyers Singleton with DI
    return instance;
  }

  void run() override {
    std::cout << "Robot running." << std::endl;
    m_behaviour->run();
  }

  explicit Robot(IBehaviour& behaviour)
      : m_behaviour(&behaviour) {
    std::cout << "Robot Constructor." << std::endl;
  }

 private:
  IBehaviour* m_behaviour;
};

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
/***
 * We can build a robot from the three components.
 * Each of those components is a singleton
 */
class RobotFactory {
 public:
  static Robot createRobot(IBoard* board, IVehicle* vehicle, IBehaviour* behaviour) {
    if (vehicle && board) {
      dynamic_cast<Vehicle*>(vehicle)->setBoard(board);
    }
    if (behaviour && vehicle) {
      dynamic_cast<Behaviour*>(behaviour)->setVehicle(vehicle);
    }
    return Robot(*behaviour);
  }
};
