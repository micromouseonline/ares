//
// Created by peter on 01/02/25.
//

#include "delay.h"
#include <cstdint>
#include "application/timer.h"
#include "vehicle/vehicle.h"

static uint32_t tickCount = 0;
static float speedUp = 1.0f;

void setSpeedUp(float s) {
  speedUp = 1.0f / s;
}

uint32_t millis() {
  return tickCount;
}

void delay_ms(uint32_t t) {
  Timer timer;
  uint32_t start = tickCount;
  while (tickCount - start < t) {
    tickCount++;
    Vehicle::instance().systick();
    timer.wait_us(1000 * speedUp);  // Avoid hogging the thread
  }
}
