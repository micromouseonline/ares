#pragma once
#include <iostream>
#include "hal/display-hal.h"

class DisplayMatrix : public DisplayHAL {
 public:
  void initialize() override {
    std::cout << "initialised the HCM3906 matrix display\n";
  };
  void cls() override {};
  void write(char c) override {};

 private:
  // device-specific display initialization and helper methods
  void init();
};
