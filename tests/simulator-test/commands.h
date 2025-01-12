//
// Created by peter on 12/01/25.
//

#pragma once

#include <map>

// Command structure
enum CommandType { ButtonPress, UpdateLEDs };
struct Command {
  CommandType type;
  int buttonID;
  bool state;
  std::map<int, bool> ledStates;
};
