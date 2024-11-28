//
// Created by peter on 25/11/2024.
//

#ifndef CORE_H
#define CORE_H

#include <SFML/Graphics.hpp>

#ifndef M_PI
#define M_PI 3.1415923846f
#endif

#define BIT(x) (1 << x)

constexpr float RADIANS = M_PI / 180.0f;
constexpr float DEGREES = 180.0f / M_PI;

inline float toRadians(float deg) {
  return deg * RADIANS;
}

inline float toDegrees(float rad) {
  return rad * DEGREES;
}

#endif  // CORE_H
