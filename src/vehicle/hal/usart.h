//
// Created by peter on 02/02/25.
//

#pragma once

/***
 * Stub for the vehicle's primary serial device
 *
 * this is n IO device so we need to implement inputs as well
 *
 * On MR32 this inherits from Print
 */

class Usart {
 public:
  void write(const char c) {
    (void)c;
  }

  int println(const char *str) {
    (void)str;
    return 0;
  }

  int puts(const char *str) {
    (void)str;
    return 0;
  }
};
