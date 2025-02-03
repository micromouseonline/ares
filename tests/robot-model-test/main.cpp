
#include <stdint.h>
#include <iostream>
#include "stdio.h"

/***
 * Simple program to model the assembly of parts of the mouse
 */

/////////////////////////////////////////////////////////////////////////////////
/***
 * The board is specific to an implementation. It has abstract drivers for any
 * anticipated hardware.
 *
 * The BoardInterface class defines some minimum set of functionality that all
 * boards must implement. So long as at least one of those is a pure virtual method
 * then it is not possible to instantiate the boardInterfacse class.
 *
 * All descendents of the BoardInterface class must implement the pure virtual methods
 * defined in the base class.
 *
 * An ideal candidate for at least one pure virtual method would be the init()
 * method that actually configures the hardware.
 *
 * Implementations of the BoardInterface class are expected to be singletons
 * and are free to add their own features.
 *
 */

class BoardInterface {
 public:
  virtual ~BoardInterface() = default;
  virtual void init() = 0;
  virtual void setMotorVolts(float left, float right) = 0;
  virtual void setLed(uint8_t id, bool state) {
    (void)id;
    (void)state;
  };

 private:
 protected:
  BoardInterface() = default;
};

class Board : public BoardInterface {
 public:
  static Board& getInstance() {
    static Board instance;  // Meyers Singleton
    return instance;
  }

  void init() override {
    std::cout << "Board initialising." << std::endl;
  }

  void setMotorVolts(float left, float right) {
    std::cout << "Board: Left Volts: " << left << ", Right Volts: " << right << std::endl;
  }

  void beep(int duration) {
    std::cout << "Board: beeping for " << duration << " milliseconds" << std::endl;
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
  explicit Vehicle(BoardInterface& board)
      : m_board(board) {
    std::cout << "\nVehicle initialized." << std::endl;
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
