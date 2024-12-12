//
// Created by peter on 25/11/2024.
//

#ifndef CORE_H
#define CORE_H

#include "SFML/Graphics.hpp"
#include "types.h"

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

/***
 * TODO Not sure this is the best place for this macro
 *
 * the macro looks odd because it uses a C++17 feature where
 * the if statement condition can include an initialiser.
 * This feature allows you to initialize variables within the
 * if statement, followed by a condition that determines if
 * the block should be executed. It's a neat way to declare
 * and initialise variables that are only used within the
 * scope of the if statement. Use it like this:
 *
 * if (initialiser; condition){
 *   // code that will run if condition is true
 * }
 *
 * The scope of any variables declared in the initializer
 * of an if statement is limited to the entire if block,
 * including both the condition and the code within the
 * braces {} of the if statement.
 *
 * Use the macro like this
 *
 * CRITICAL_SECTION(m_systick_mutex){
 *   // guarded code
 * }
 *
 */

#define CRITICAL_SECTION(the_mutex) if (std::lock_guard<std::mutex> lock(the_mutex); true)

#endif  // CORE_H
