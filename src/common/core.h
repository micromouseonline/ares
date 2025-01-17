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
#include "SFML/Graphics.hpp"
#include "application/types.h"

constexpr float kPI = 3.14159265358979323846f;  // More precise

constexpr float kEpsilon = std::numeric_limits<float>::epsilon();

constexpr int BIT(int x) {
  return 1 << x;
}

inline float sign(float a) {
  return a >= 0 ? 1.0f : -1.0f;
}

constexpr float RADIANS = kPI / 180.0f;
constexpr float DEGREES = 180.0f / kPI;

inline float toRadians(float deg) {
  return deg * RADIANS;
}

inline float toDegrees(float rad) {
  return rad * DEGREES;
}

inline std::atomic<uint32_t> g_ticks = 0;

/// TODO - use a thread-safe class for this
inline std::mutex g_log_mutex;  // Protects the global log

inline std::queue<std::string> g_log_messages;
inline void logMessage(const std::string msg) {
  std::lock_guard<std::mutex> lock(g_log_mutex);
  g_log_messages.push(msg);
}

/** * CRITICAL_SECTION
 *
 * The macro uses a C++17 feature where the if statement condition can include an
 * initializer.
 * This allows you to initialize variables within the if statement, followed
 * by a condition that determines if the block should be executed.
 *
 * Usage:
 * CRITICAL_SECTION(m_systick_mutex) {
 *     // guarded code
 * }
 */
#define CRITICAL_SECTION(the_mutex) if (std::lock_guard<std::mutex> lock(the_mutex); true)

#endif  // CORE_H
