//
// Created by peter on 03/02/25.
//

#pragma once

/////////////////////////////////////////////////////////////////////////////////
/***
 *
 * Defines the interface for a board. as well as a basic derived class called
 * the BasicBoard.
 *
 * Basicboard is the default implementation of the BoardInterface class. It
 * does nothing but serve as a placeholder for development and testing.
 *
 * Each board class is specific to an implementation. It has abstract drivers for any
 * anticipated hardware.
 *
 * The BoardInterface class defines some minimum set of functionality that all
 * boards must implement. by making the init() method a pure virtual method
 * then it is not possible to instantiate the boardInterfacse class.
 *
 * All descendents of the BoardInterface class must implement the pure virtual methods
 * defined in the base class.
 *
 * Implementations of the BoardInterface class are free to add their own features.
 *
 * Implementations can be singletons if that is desired but it is not required.
 *
 */

#include <stdint.h>
#include <iostream>
class BoardInterface {
 public:
  virtual ~BoardInterface() = default;
  virtual void init() = 0;
  virtual void update() = 0;
  virtual void setLed(int id, bool state) {
  }
  virtual bool isButtonPressed(int button_id) {
    return false;
  }
  virtual bool isAnyButtonPressed() {
    return false;
  }
  virtual void beep(int duration) {
  }
  virtual void playTone(uint32_t frequency, uint32_t duration) {
  }
  virtual void serialWrite(const char c) {
  }
  virtual void serialPuts(const char* str) {
  }
  virtual void displayWrite(const char c) {
  }
  virtual void displayPuts(const char* str) {
  }
  virtual void displayClear() {
  }
  virtual uint16_t getAdcChannel(uint8_t channel) {
    return 0;
  }
  virtual float getBatteryVoltage() {
    return 0.0f;
  }
  virtual void setMotorVoltage(float left, float right) {};
  virtual int getLeftEncoderCount() {
    return 0;
  }
  virtual int getRightEncoderCount() {
    return 0;
  }
  virtual void resetEncoders() {
  }
  virtual float getImuYaw() {
    return 0.0f;
  }
  virtual void resetImu() {
  }

 protected:
  /// only derived classes can call the constructor
  BoardInterface() = default;

 private:
  // Delete copy/move constructors and assignment operators
  BoardInterface(const BoardInterface&) = delete;
  BoardInterface& operator=(const BoardInterface&) = delete;
  BoardInterface(BoardInterface&&) = delete;
  BoardInterface& operator=(BoardInterface&&) = delete;
};

/////////////////////////////////////////////////////////////////////////////////
/***
 * The Basicboard is provided as a non-functional example of an actual board
 * implementation. It overrides only the init() method which will be called
 * on instantiation and write a message to stdout.
 *
 * BasicBoard is a singleton and you can instantiate it with or without
 * parameters by supplying a void*.
 */
class BasicBoard : public BoardInterface {
 public:
  static BasicBoard& getInstance(void* params = nullptr) {
    static BasicBoard instance(params);  // Meyers Singleton
    return instance;
  }

  void init() override {
    std::cout << "initialising." << std::endl;
  }
  void init(void* params) {
    std::cout << "initialising with parameters." << std::endl;
  }

  void update() override {
    std::cout << "updating." << std::endl;
  }

 private:
  /// you must provide the constructor
  BasicBoard(void* params = nullptr) {
    std::cout << "BasicBoard Constructor with parameters... ";
    if (params) {
      init(params);
    } else {
      init();
    }
  }
  /// You might need shutdown code. e.g. close a logger.
  ~BasicBoard() = default;
};
