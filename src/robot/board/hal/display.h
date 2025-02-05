//
// Created by peter on 02/02/25.
//

#pragma once

/**
 * Stub for an on-board display
 */
class Display {
 public:
  void write(uint8_t c) {
    (void)c;
  };

  void printf(const char *format, ...) {
    (void)format;
  };

  int puts(const char *str) {
    (void)str;
    return 0;
  };

  void cls() {
  }
};
