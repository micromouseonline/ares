//
// Created by peter on 12/01/25.
//

#pragma once

#include <cstdio>
#include <string>
#include "common/expfilter.h"
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
  float filter_alpha = 0.90f;

#define LOG_BUFFER_SIZE 1024
  char logBuffer[LOG_BUFFER_SIZE];  // Array of 32-character strings
  volatile int logIndex = 0;

  Target() : sensorCallback(nullptr), battery(0.95) {
    setup();
    logBuffer[logIndex++] = '\0';
    printf("Target setup\n");
  }

  ~Target() {
    printf("Target cleanup\n");
  }

  // Timer setup for 500Hz simulated using a member function
  void timerISR() {
    Timer timer;
    ticks += 2;
    if (sensorCallback) {
      sensors = sensorCallback(12);
    }

    sensors.battery = battery.update(50 + random() % 30);
    log("systick  log ");
    timer.wait_us(1000 * 2);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(2));  // Simulate 500Hz timer
  }

  void delay_ms(uint32_t ms) {
    uint32_t end = ticks + ms;
    while (ticks < end) {
      timerISR();
    }
  }

  void logTicks() {
    if (logIndex > LOG_BUFFER_SIZE - 12) {
      return;
    }
    char* p = logBuffer + logIndex;
    int len = snprintf(p, 10, "%7u ", ticks);
    logIndex += len;
  }

  /***
   * Logged messages automatically get a timestamp prepended and are
   * null terminated
   * @param message
   */
  void log(const char* message) {
    int messageLength = strlen(message);
    // Ensure the message fits in the remaining space in the buffer
    if ((messageLength + logIndex + 10) >= LOG_BUFFER_SIZE - 1) {
      return;
    }
    logTicks();
    strncpy(logBuffer + logIndex, message, messageLength);
    logIndex += messageLength;
    logBuffer[logIndex++] = '\0';  // Null-terminate the message
  }

  /// Copy the logging buffer. Do not bother if it is empty
  bool getLogBuffer(char* buffer) {
    if (logBuffer[0] == '\0') {
      return false;
    }
    memcpy(buffer, logBuffer, LOG_BUFFER_SIZE);
    buffer[LOG_BUFFER_SIZE - 1] = '\0';
    clearLogBuffer();
    return true;
  }

  // Add a method to clear the log buffer
  void clearLogBuffer() {
    memset(logBuffer, 0, LOG_BUFFER_SIZE);  // Reset the buffer
    logIndex = 0;                           // Reset the index to the beginning
  }

  void setup() {
    for (auto& pin : pins) {
      pin = HIGH;
    }
    ticks = 0;
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
