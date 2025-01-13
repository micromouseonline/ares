//
// Created by peter on 12/01/25.
//

#pragma once

#include <cstdio>
#include <string>
#include "common/expfilter.h"

struct SensorData {
  int lfs;
  int lds;
  int rds;
  int rfs;
  float battery;
  SensorData() : lfs(0), lds(0), rds(0), rfs(0), battery(78){};
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
  bool is_running = true;
  bool paused = false;
  volatile bool pins[16];
  volatile uint32_t ticks;
  SensorData sensors;
  SensorCallbackFunction sensorCallback;
  ExpFilter<float> battery;
  std::queue<std::string> log_buffer;

  Target() : sensorCallback(nullptr), battery(0.95) {
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
      sensors = sensorCallback(12);
    }

    sensors.battery = battery.update(50 + random() % 30);
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
    log("Target Ready");
  }

  void stopRunning() {
    is_running = false;
  }

  void pauseRunning() {
    log("Paused");
    paused = true;
  }

  void resumeRunning() {
    log("Resumed");
    paused = false;
  }

  void log(const std::string msg) {
    log_buffer.push(msg);
  }

  std::queue<std::string> getLogs() {
    return log_buffer;
  }

  void mainLoop() {
    uint32_t interval = 500;
    uint32_t next_update = ticks + interval;
    while (is_running) {
      if (paused) {
        continue;
      }
      if (digitalRead(11) == LOW) {
        digitalWrite(12, !digitalRead(12));
        digitalWrite(11, true);
        log("toggled pin 12");
      }
      if (ticks >= next_update) {
        next_update += interval;
        pins[0] = !pins[0];
      }
      delay_ms(2);  // get at least one tick in every cycle
    }
  }

  void setSensorCallback(SensorCallbackFunction callback) {
    sensorCallback = callback;
  }

  SensorData getSensors() {
    return sensors;
  }

  std::array<bool, 16> getPinState() const {
    std::array<bool, 16> pin_states;
    for (int i = 0; i < 16; i++) {
      pin_states[i] = pins[i];
    }
    return pin_states;
  }

  bool digitalRead(int pin) {
    return pins[pin];
  }

  void digitalWrite(int pin, bool state) {
    pins[pin] = state;
  }
};
