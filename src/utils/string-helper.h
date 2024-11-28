//
// Created by peter on 22/11/24.
//

#ifndef IMGUI_SFML_STARTER_STRING_HELPER_H
#define IMGUI_SFML_STARTER_STRING_HELPER_H

#include <sstream>

// Since std::to_string doesn't work on MinGW we have to implement
// our own to support all platforms.
template <typename T>
std::string toString(const T& value);

template <typename T>
std::string toString(const T& value) {
  std::stringstream stream;
  stream << value;
  return stream.str();
}
#endif  // IMGUI_SFML_STARTER_STRING_HELPER_H
