#pragma once
#include "display-hal.h"

class DisplayOLED : public DisplayHAL {
 public:
  void initialize() override {};
  void cls() override {};
  void write(char c) override {};

 private:
  // device-specific display initialization and helper methods
  void init();
};
