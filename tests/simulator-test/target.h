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
  volatile bool pins[16];
  volatile uint32_t ticks;
  SensorData sensors;
  SensorCallbackFunction sensorCallback;

  Target() : sensorCallback(nullptr) {
    setup();
    printf("Target setup\n");
  }

  ~Target() {
    printf("Target cleanup\n");
  }

  // Timer setup for 500Hz simulated using a member function
  void timerISR() {
    ticks += 2;
    if (sensorCallback) {
      sensors = sensorCallback(10);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));  // Simulate 500Hz timer
  }

  void delay_ms(uint32_t ms) {
    uint32_t end = ticks + ms;
    while (ticks < end) {
      timerISR();
    }
  }

  void setup() {
    for (auto& pin : pins) {
      pin = HIGH;
    }
  }

  void mainLoop() {
    while (true) {
      if (digitalRead(10) == LOW) {
        printf("Gotcha\n");
        while (digitalRead(10) == LOW) {
          delay_ms(2);
        }
      }
      delay_ms(2);  // get at least one tick in every cycle
    }
  }

  void setSensorCallback(SensorCallbackFunction callback) {
    sensorCallback = callback;
  }

  std::map<int, bool> getPinState() {
    return {
        {4, pins[4]},  //
        {5, pins[5]}   //
    };
  }

  void simulateButtonPress(int pin) {
    pins[pin] = !pins[pin];
    //    printf("Button press %d\n", pin);
  }

  bool digitalRead(int pin) {
    return pins[pin];
  }

  void digitalWrite(int pin, bool state) {
    pins[pin] = state;
    /// the message will be laggy
    printf("pin %d set to %s\n", pin, state ? "HIGH" : "LOW");
  }
};
