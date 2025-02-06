//
// Created by peter on 05/02/25.
//

#pragma once

#include "hal/encoder-hal.h"

class Encoder {
 public:
  explicit Encoder(EncoderHAL& hal)
      : m_encoder_hal(hal) {
  }

  int getTicks() const {
    return m_encoder_hal.getCount();
  }

  float getSpeed() const {
    return m_encoder_hal.getChange();
  }

  void reset() {
    m_encoder_hal.clear();
  }

 private:
  EncoderHAL& m_encoder_hal;
};
