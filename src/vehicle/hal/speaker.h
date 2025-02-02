//
// Created by peter on 01/02/25.
//
#pragma once

#include "common/delay.h"

/**
 * A speaker interface class
 *
 * There are no real speakers in the simulator and there seem to be a lot
 * of specific methods. It might be best to reduce these to the bare minimum
 *
 * Meanwhile it might be a good idea if the issue a message or log entry to
 * show they have been called. The simulator could actually d i the sounds I guess
 */

class Speaker {
 public:
  void begin() {
    /// sets up the hardware
  }

  void on() {
    /// connects the timer to the output pin
  }

  void off() {
    /// diconnects the timer from the output pin
  }

  /// called from systick to update the speaker
  void update() {
  }

  void playTone(uint32_t frequency, uint32_t duration) {
    delay_ms(duration);
  }

  void confirm() {
  }

  void k() {
  }

  void exclaim() {
  }

  void good() {
  }

  void alert() {
  }

  void cricket(uint32_t pattern) {
    (void)pattern;
  }

  void trill() {
  }

  void complete() {
  }

  void success() {
  }

  void fail() {
  }

  void error() {
  }
};
