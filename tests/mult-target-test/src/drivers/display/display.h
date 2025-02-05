#pragma once

#include <string>
#include "display-hal.h"

/// NOTE it is not clear this is needed unless I want a specific display from a board
///      where there are choices
class Display {
 public:
  // Constructor takes a reference to a DisplayHAL interface (dependency injection)
  explicit Display(DisplayHAL& displayHAL)
      : m_displayHAL(displayHAL) {

        };

  // Initializes the display
  void initialize() {
    m_displayHAL.initialize();
  };

  // Clears the display
  void cls() {
    m_displayHAL.cls();
  };

  // Writes a single character to the display
  void write(char c) {
    m_displayHAL.write(c);
  };

 private:
  DisplayHAL& m_displayHAL;  // The hardware abstraction layer for the display
};
