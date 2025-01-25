//
// Created by peter on 25/11/2024.
//

#ifndef CORE_H
#define CORE_H

#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <queue>
// #include "SFML/Graphics.hpp"
#include "application/sensor-geometry.h"

#ifndef kPI
#define kPI 3.14159265358979323846f  // More precise
#define RADIANS (kPI / 180.0f)
#define DEGREES (180.0f / kPI)
#endif

#ifndef BIT
#define BIT(b) (1UL << (b))
#endif

#ifndef SIGN
#define SIGN(x) ((0 < x) - (x < 0))
#endif

/// TODO - use a thread-safe class for this
inline std::mutex g_log_mutex;  // Protects the global log

inline std::queue<std::string> g_log_messages;
inline void logMessage(const std::string msg) {
  std::lock_guard<std::mutex> lock(g_log_mutex);
  g_log_messages.push(msg);
}

inline float normalizeAngle(float angle) {
  angle = fmod(angle + 180.0f, 360.0f);
  if (angle < 0) {
    angle += 360.0f;
  }
  if (angle == -180.0f) {
    angle = 180.0f;
  }
  return angle - 180.0f;
}

using SerialOut = std::function<void(const char)>;
using BinaryOut = std::function<void(const uint8_t)>;

#endif  // CORE_H
