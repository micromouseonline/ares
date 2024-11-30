//
// Created by peter on 17/04/24.
//

#ifndef IMGUI_SFML_STARTER_EXPFILTER_H
#define IMGUI_SFML_STARTER_EXPFILTER_H

template <class T>
class ExpFilter {
 public:
  T value;
  float m_alpha;
  explicit ExpFilter(float alpha = 0.5) : m_alpha(alpha){};
  T update(T x) {
    value = m_alpha * value + (1 - m_alpha) * x;
    return value;
  }
};

#endif  // IMGUI_SFML_STARTER_EXPFILTER_H
