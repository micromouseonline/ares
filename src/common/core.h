//
// Created by peter on 25/11/2024.
//

#ifndef CORE_H
#define CORE_H

#include <cmath>
#include <limits>
#include <mutex>
#include "SFML/Graphics.hpp"
#include "types.h"

constexpr float kPI = 3.14159265358979323846f;  // More precise

constexpr float kEpsilon = std::numeric_limits<float>::epsilon();

constexpr int BIT(int x) {
  return 1 << x;
}

constexpr float RADIANS = kPI / 180.0f;
constexpr float DEGREES = 180.0f / kPI;

inline float toRadians(float deg) {
  return deg * RADIANS;
}

inline float toDegrees(float rad) {
  return rad * DEGREES;
}

/// Shared mutexes for access to the behaviour and robot from the application
inline std::mutex g_robot_mutex;      // Protects access to m_pose and m_orientation
inline std::mutex g_mutex_obstacles;  // Protects access when building collision list

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
