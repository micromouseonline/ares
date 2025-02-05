#pragma once
#include <iostream>
#include "hal/display-hal.h"

class DisplayOLED : public DisplayHAL {
 public:
  void initialize() override {
    std::cout << "initialised the OLED\n";
  };
  void cls() override {};
  void write(char c) override {};
  void setCursor(int x, int y) override {};

 private:
  // device-specific display initialization and helper methods
  void init() {};
};
