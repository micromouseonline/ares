//
// Created by peter on 12/01/25.
//

#pragma once

// #include <cstdio>
#include <string>
#include "common/expfilter.h"
#include "common/printf/printf.h"
#include "common/queue.h"
#include "common/timer.h"

void _putchar(char c) {
  std::cout << c;
}

// Simulated Arduino Nano Target class
class Target {
 public:
  bool is_running = true;
  volatile uint32_t ticks;

  using SerialCallback = std::function<void(const char)>;
  SerialCallback serialOut;

  Target() : ticks(0), serialOut(nullptr) {
    printf("Target setup\n");
  }

  ~Target() {
    printf("Target cleanup\n");
  }

  void setSerialoutCallback(SerialCallback cb) {
    serialOut = cb;
  }

  // Timer setup for 500Hz simulated using a member function
  void timerISR() {
    Timer timer;
    ticks += 1;
    timer.wait_us(1000);
  }

  void delay_ms(uint32_t ms) {
    uint32_t end = ticks + ms;
    while (ticks < end) {
      timerISR();
    }
  }

  void log(const char* message) {
    char buf[128];
    snprintf(buf, 126, "%7u %s", ticks, message);
    std::cout << buf;
    return;
  }

  void stopRunning() {
    is_running = false;
  }

  void mainLoop() {
    uint32_t interval = 1000;
    uint32_t next_update = ticks + interval;
    while (is_running) {
      if (ticks >= next_update) {
        log("update the blinker\n");
        next_update += interval;
      }
      delay_ms(1);
    }
    delay_ms(1);  // get at least one tick in every cycle
  }
};
