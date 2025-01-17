//
// Created by peter on 12/01/25.
//

#pragma once

// #include <cstdio>
#include <string>
#include "behaviour/expfilter.h"
#include "common/printf/printf.h"
#include "common/queue.h"
#include "common/timer.h"

struct SensorData {
  int lfs;
  int lds;
  int rds;
  int rfs;
  float battery;
  SensorData() : lfs(0), lds(0), rds(0), rfs(0), battery(78) {};
};

typedef std::function<SensorData(int)> SensorCallbackFunction;

// Constants
const int INPUT = 0;
const int OUTPUT = 1;
const bool HIGH = true;
const bool LOW = false;

void _putchar(char c) {
  std::cout << c;
}
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
  float filter_alpha = 0.90f;
  using SerialCallback = std::function<void(const char*)>;
  SerialCallback serialOut;

  Target() : sensorCallback(nullptr), battery(0.95) {
    setup();
    printf("Target setup\n");
  }

  ~Target() {
    printf("Target cleanup\n");
  }

  void setLogCallback(SerialCallback cb) {
    serialOut = cb;
  }

  void setSensorCallback(SensorCallbackFunction callback) {
    sensorCallback = callback;
  }

  // Timer setup for 500Hz simulated using a member function
  void systick() {
    Timer timer;
    ticks += 1;
    if (sensorCallback) {
      sensors = sensorCallback(12);
    }

    sensors.battery = battery.update(50 + random() % 30);
    log("systick  log 0123456789012345678901234567890123456789 0123456789");
    timer.wait_us(1000);
  }

  void delay_ms(uint32_t ms) {
    uint32_t end = ticks + ms;
    while (ticks < end) {
      systick();
    }
  }

  /***
   * Logged messages automatically get a timestamp prepended and are
   * null terminated
   * @param message
   */
  void log(const char* message) {
    char buf[128];
    snprintf(buf, 126, "%7u %s", ticks, message);
    if (serialOut) {
      serialOut(buf);
    }
    return;
  }

  void setup() {
    for (auto& pin : pins) {
      pin = HIGH;
    }
    ticks = 0;
    log("Target Ready");
  }

  ////////////////////////////////////
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
  ////////////////////////////////////

  void setFilterAlpha(float alpha) {
    filter_alpha = alpha;
    battery.m_alpha = filter_alpha;
    char buf[64];
    snprintf(buf, 60, "Filter set to %5.3f\n", alpha);
    log(buf);
  }

  float getFilterAlpha() {
    return filter_alpha;
  }

  void mainLoop() {
    uint32_t interval = 500;
    uint32_t next_update = ticks + interval;
    while (is_running) {
      if (paused) {
        continue;
      }
      //      log("check pin 11");
      if (!digitalRead(11)) {
        log("Pin 11 was high");
        digitalWrite(12, !digitalRead(12));
        digitalWrite(11, true);
        log("toggled pin 12");
      }
      if (ticks >= next_update) {
        next_update += interval;
        log("update the blinker");
        pins[0] = !pins[0];

        for (int i = 0; i < 8; i++) {
          delay_ms(4);
          char buf[32];
          sprintf(buf, "... loop %d", i);
          log(buf);
        }
      }

      delay_ms(2);  // get at least one tick in every cycle
    }
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
