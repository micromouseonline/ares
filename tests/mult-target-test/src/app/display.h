#pragma once

#include <string>
#include "hal/display-hal.h"

/***
 * Provide a generic interface for a display that can be instantiated with a specific HAL
 * implementation.
 *
 * The DisplayHAL class should be about low-level control of the display, providing simple
 * methods for writing characters and strings as well as basic operations like clearing
 * the screen and positioning the cursor. Graphics displays might add line drawing methods.
 * the only thing that is required is the ability to write characters and send commands.
 *
 * The Display class takes a higher level view of the display and may provide methods like
 * text alignment and formatting. Or it may, for example, be able to handle high level
 * data sources like std::strings
 *
 * As a form of dependency injection, this is also be useful for testing and would make it
 * easier to change the display implementation. For example a single board may have
 * multiple options for the physical device attached. This would allow the user to
 * specify which device to use.
 *
 */
class Display {
 public:
  // Constructor takes a reference to a DisplayHAL interface which is a generic interface

  explicit Display(DisplayHAL& displayHAL)
      : m_displayHAL(displayHAL) {};

  void initialize() {
    m_displayHAL.initialize();
  };

  void cls() {
    m_displayHAL.cls();
  };

  void setCursor(int x, int y) {
    m_displayHAL.setCursor(x, y);
  }

  void writeString(const std::string str) {
    for (char c : str) {
      m_displayHAL.write(c);
    }
  };

 private:
  DisplayHAL& m_displayHAL;  // The hardware abstraction layer for the display
};
