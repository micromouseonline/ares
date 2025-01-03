//
// Created by peter on 17/04/24.
//

#pragma once

template <class T>
class ExpFilter {
 public:
  T value;
  float m_alpha;
  explicit ExpFilter(float alpha = 0.5) : m_alpha(alpha) {};
  T update(T x) {
    value = m_alpha * value + (1 - m_alpha) * x;
    return value;
  }
};
