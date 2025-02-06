//
// Created by peter on 05/02/25.
//

#pragma once

#include <stdint.h>
#include "hal/encoder-hal.h"
// #include "stm32/gpio.h"

class STM32Encoder : public EncoderHAL {
 public:
  STM32Encoder(int32_t& counter)
      : m_register(counter),
        tickCount(0) {
  }

  void initialize() override {};

  int32_t getCount() override {
    return tickCount;
  }

  int32_t getChange() override {
    return tickCount;
  }

  void clear() override {
    tickCount = 0;
  }

  void update() {
    uint32_t count = m_register;
    m_register = 0;
    tickCount += count;
  }

 private:
  uint32_t m_register;  /// fake hardware counter register for illustration
  int tickCount;
};
