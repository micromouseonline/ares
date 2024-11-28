//
// Created by peter on 07/11/24.
//

#ifndef IMGUI_SFML_STARTER_PVECTOR_H
#define IMGUI_SFML_STARTER_PVECTOR_H

#include <cassert>
#include <cmath>
#include <iostream>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class PVector {
 public:
  float x;
  float y;

  static constexpr float EPSILON = 1e-6;

  // Constructors
  explicit PVector(float x = 0, float y = 0) : x(x), y(y) {}

  static PVector from_angle(float angle) { return PVector(cosf(angle), sinf(angle)); }

  static PVector zero() { return PVector(0, 0); }
  static PVector up() { return PVector(0, 1); }
  static PVector right() { return PVector(1, 0); }

  PVector random() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0, 2 * M_PI);
    return from_angle(dist(gen));
  }

  PVector left_normal() const {
    PVector v(-y, x);
    return v;
  }

  PVector right_normal() const {
    PVector v(y, -x);
    return v;
  }

  // Add two vectors
  PVector operator+(const PVector& v) const { return PVector(x + v.x, y + v.y); }

  // Subtract two vectors
  PVector operator-(const PVector& v) const { return PVector(x - v.x, y - v.y); }

  // Multiply vector by a scalar
  PVector operator*(float n) const { return PVector(x * n, y * n); }

  // Divide vector by a scalar
  PVector operator/(float n) const {
    assert(std::fabs(n) > EPSILON && "Division by zero is not allowed.");
    float reciprocal = 1.0f / n;
    return PVector(x * reciprocal, y * reciprocal);
  }

  // Compound assignment operators
  PVector& operator+=(const PVector& v) {
    x += v.x;
    y += v.y;
    return *this;
  }

  PVector& operator-=(const PVector& v) {
    x -= v.x;
    y -= v.y;
    return *this;
  }

  PVector& operator*=(float n) {
    x *= n;
    y *= n;
    return *this;
  }

  PVector& operator/=(float n) {
    assert(std::fabs(n) > EPSILON && "Division by zero is not allowed.");
    x /= n;
    y /= n;
    return *this;
  }

  bool operator==(const PVector& v) const { return (std::fabs(x - v.x) < EPSILON) && (std::fabs(y - v.y) < EPSILON); }

  // Calculate the magnitude of the vector
  float mag() const { return std::sqrt(x * x + y * y); }

  // Normalize the vector to a unit vector
  PVector& normalize() {
    float m = mag();
    if (m > EPSILON) {
      *this /= m;
    }
    return *this;
  }

  PVector& set_magnitude(float m) {
    normalize();
    *this *= m;
    return *this;
  }

  PVector& limit(float lim) {
    if (mag() > lim) {
      set_magnitude(lim);
    }
    return *this;
  }

  // Calculate the dot product of two vectors
  float dot(const PVector& v) const { return x * v.x + y * v.y; }

  float angle() { return atan2f(y, x); }

  float angle_to(PVector& v2) { return acosf(dot(v2) / (mag() * v2.mag())); }

  float signed_angle_to(const PVector& v) const { return atan2f(v.y, v.x) - atan2f(y, x); }

  float cross(const PVector& v) const { return x * v.y - y * v.x; }

  PVector& rotate_to(float theta) {
    float m = mag();
    x = m * cosf(theta);
    y = m * sinf(theta);
    return *this;
  }

  PVector& rotate_by(float theta) {
    rotate_to(angle() + theta);
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os, const PVector& v) {
    os << "PVector(" << v.x << ", " << v.y << ")";
    return os;
  }
};

#endif  // IMGUI_SFML_STARTER_PVECTOR_H
