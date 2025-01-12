//
// Created by peter on 12/01/25.
//

#pragma once

#include <cstdio>

struct SensorData {
  int lfs;
  int lds;
  int rds;
  int rfs;
};

typedef std::function<SensorData(int)> SensorCallbackFunction;

// Constants
const int INPUT = 0;
const int OUTPUT = 1;
const bool HIGH = true;
const bool LOW = false;

// Simulated Arduino Nano Target class
class Target {
 public:
  volatile bool led10;
  volatile bool led11;
  SensorData sensors;
  SensorCallbackFunction sensorCallback;

  Target() : led10(false), led11(false), sensorCallback(nullptr) {
    setup();
    printf("Target setup\n");
  }

  ~Target() {
    printf("Target cleanup\n");
  }

  // Timer setup for 500Hz simulated using a member function
  void timerISR() {
    if (sensorCallback) {
      sensors = sensorCallback(10);
    }
  }

  void setup() {
    pinMode(10, INPUT);
    pinMode(11, INPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);

    // Timer setup - use a different mechanism to simulate the timer interrupts in the simulation environment
    std::thread timerThread([this]() {
      while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));  // Simulate 500Hz timer
        timerISR();
      }
    });
    timerThread.detach();
  }

  void loop() {
    while (true) {
      digitalWrite(4, led10 ? HIGH : LOW);
      digitalWrite(5, led11 ? HIGH : LOW);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Add some delay for stability
    }
  }

  void setSensorCallback(SensorCallbackFunction callback) {
    sensorCallback = callback;
  }

  std::map<int, bool> getLEDStates() {
    return {{4, led10}, {5, led11}};
  }

  void simulateButtonPress(int buttonID) {
    if (buttonID == 10) {
      led10 = !led10;
    } else if (buttonID == 11) {
      led11 = !led11;
    }
  }

  int digitalRead(int pin) {
    return (pin == 10) ? led10 : led11;
  }

  void digitalWrite(int pin, bool state) {
    printf("pin %d set to %s\n", pin, state ? "LOW" : "HIGH");
  }

  void pinMode(int pin, int mode) {
    printf("pin %d set to mode %s\n", pin, mode ? "OUTPUT" : "INPUT");
  }

  void noInterrupts() {
  }
  void interrupts() {
  }
};
