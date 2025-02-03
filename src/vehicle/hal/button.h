//
// Created by peter on 02/02/25.
//

#pragma once

/***
 * Stub for a single button
 *
 */

class Button {
 public:
  bool isPressed() {
    return m_state;
  }

  int getPressType() {
    return 1;
  }

 private:
  bool m_state;
};
