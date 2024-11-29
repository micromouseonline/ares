//
// Created by peter on 18/11/24.
//

#ifndef VEC2_H
#define VEC2_H

#include <SFML/Graphics.hpp>
#include <cassert>
#include <cmath>

struct Vec2 {
  float x, y;
  static constexpr float EPSILON = 1e-6;

  Vec2() : x(0.0f), y(0.0f) {}
  Vec2(float x_, float y_) : x(x_), y(y_) {}
  explicit Vec2(const sf::Vector2f& v) : x(v.x), y(v.y) {}
  explicit Vec2(const sf::Vector2i& v) : x((float)v.x), y((float)v.y) {}
  explicit Vec2(const sf::Vector2u& v) : x((float)v.x), y((float)v.y) {}

  static Vec2 fromAngle(float angle) { return {cosf(angle), sinf(angle)}; }

  static Vec2 zero() { return {0, 0}; }
  static Vec2 up() { return {0, 1}; }
  static Vec2 right() { return {1, 0}; }

  /// convert to sf::Vector2f
  explicit operator sf::Vector2f() const { return {x, y}; }
  /// convert to sf::Vector2i
  explicit operator sf::Vector2i() const { return {(int)x, (int)y}; }
  /// convert to sf::Vector2u  - POTENTIALLY DANGEROUS - DROPS SIGN
  explicit operator sf::Vector2u() const { return {(unsigned int)x, (unsigned int)y}; }

  [[nodiscard]] float length() const { return sqrtf(x * x + y * y); }

  [[nodiscard]] Vec2 getNormalized() const {
    const float len = length();
    return {x / len, y / len};
  }

  [[nodiscard]] Vec2 normal() const { return {-y, x}; }

  [[nodiscard]] Vec2 left_normal() const {
    Vec2 v(-y, x);
    return v;
  }

  [[nodiscard]] Vec2 right_normal() const {
    Vec2 v(y, -x);
    return v;
  }

  // Add two vectors
  Vec2 operator+(const Vec2& v) const { return {x + v.x, y + v.y}; }

  // Subtract two vectors
  Vec2 operator-(const Vec2& v) const { return {x - v.x, y - v.y}; }

  // Multiply vector by a scalar
  Vec2 operator*(float n) const { return {x * n, y * n}; }

  // Divide vector by a scalar
  Vec2 operator/(float n) const {
    assert(std::fabs(n) > EPSILON && "Division by zero is not allowed.");
    float reciprocal = 1.0f / n;
    return Vec2(x * reciprocal, y * reciprocal);
  }

  // Compound assignment operators
  Vec2& operator+=(const Vec2& v) {
    x += v.x;
    y += v.y;
    return *this;
  }

  Vec2& operator-=(const Vec2& v) {
    x -= v.x;
    y -= v.y;
    return *this;
  }

  Vec2& operator*=(float n) {
    x *= n;
    y *= n;
    return *this;
  }

  Vec2& operator/=(float n) {
    assert(std::fabs(n) > EPSILON && "Division by zero is not allowed.");
    x /= n;
    y /= n;
    return *this;
  }

  bool operator==(const Vec2& v) const { return (std::fabs(x - v.x) < EPSILON) && (std::fabs(y - v.y) < EPSILON); }

  // Calculate the magnitude of the vector
  float mag() const { return std::sqrt(x * x + y * y); }

  // Normalize the vector to a unit vector
  Vec2& normalize() {
    float m = mag();
    if (m > EPSILON) {
      *this /= m;
    }
    return *this;
  }

  Vec2& set_magnitude(float m) {
    normalize();
    *this *= m;
    return *this;
  }

  Vec2& limit(float lim) {
    if (mag() > lim) {
      set_magnitude(lim);
    }
    return *this;
  }

  // Calculate the dot product of two vectors
  float dot(const Vec2& v) const { return x * v.x + y * v.y; }

  float dot(const Vec2& v1, const Vec2& v2) { return v1.x * v2.x + v1.y * v2.y; }

  float angle() { return atan2f(y, x); }

  float angle_to(Vec2& v2) { return acosf(dot(v2) / (mag() * v2.mag())); }

  float signed_angle_to(const Vec2& v) const { return atan2f(v.y, v.x) - atan2f(y, x); }

  float cross(const Vec2& v) const { return x * v.y - y * v.x; }

  Vec2& rotate_to(float theta) {
    float m = mag();
    x = m * cosf(theta);
    y = m * sinf(theta);
    return *this;
  }

  Vec2& rotate_by(float theta) {
    rotate_to(angle() + theta);
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os, const Vec2& v) {
    os << "PVector(" << v.x << ", " << v.y << ")";
    return os;
  }

  static float getDistance(const Vec2& v1, const Vec2& v2) { return (v1 - v2).length(); }

  // line is from v to w
  // point is p
  inline float minimum_distance(Vec2 v, Vec2 w, Vec2 p) {
    // Return minimum distance between line segment vw and point p
    Vec2 line = w - v;
    const float l2 = line.length();  // i.e. |w-v|^2 -  avoid a sqrt
    if (std::fabs(l2) <= std::numeric_limits<float>::epsilon())
      return getDistance(p, v);  // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line.
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    // We clamp t from [0,1] to handle points outside the segment vw.
    const float t = std::max(0.0f, std::min(1.0f, dot(p - v, w - v) / l2));
    const Vec2 projection = v + line * t;  // Projection falls on the segment
    return getDistance(p, projection);
  }
};

#endif  // VEC2_H
