//
// Created by peter on 22/11/24.
//

#ifndef DRAWING_H
#define DRAWING_H

#include <SFML/Graphics.hpp>

inline void draw_line(sf::RenderTarget& target, sf::Vector2f start, sf::Vector2f end, sf::Color color = sf::Color::White) {
  sf::Vertex line[2];
  line[0].position = start;
  line[0].color = color;
  line[1].position = end;
  line[1].color = color;
  target.draw(line, 2, sf::Lines);
}

inline void draw_vector(sf::RenderTarget& target, sf::Vector2f pos, sf::Vector2f vec, sf::Color color = sf::Color::White) {
  sf::Vertex line[2];
  line[0].position = pos;
  line[0].color = color;
  line[1].position = pos + vec;
  line[1].color = color;
  target.draw(line, 2, sf::Lines);
}

// Function to draw a vector with an arrowhead
#include <SFML/Graphics.hpp>
#include <cmath>

// Function to draw a vector with an arrowhead
inline void draw_vector_arrow(sf::RenderTarget& target, sf::Vector2f pos, sf::Vector2f vec, float arrowheadSizePercent, sf::Color color = sf::Color::White) {
  sf::Vertex line[2];
  line[0].position = pos;
  line[0].color = color;
  line[1].position = pos + vec;
  line[1].color = color;

  target.draw(line, 2, sf::Lines);

  // Arrowhead
  float arrowLength = std::sqrt(vec.x * vec.x + vec.y * vec.y) * (arrowheadSizePercent / 100.0f);  // Arrowhead length as a percentage of the vector length
  float arrowAngle = 30.0f;                                                                        // Angle between the arrowhead lines and the vector

  // Calculate the direction of the vector
  float angle = std::atan2(vec.y, vec.x);
  sf::Vector2f arrowVec1(arrowLength * std::cos(angle - arrowAngle * M_PI / 180.0f), arrowLength * std::sin(angle - arrowAngle * M_PI / 180.0f));
  sf::Vector2f arrowVec2(arrowLength * std::cos(angle + arrowAngle * M_PI / 180.0f), arrowLength * std::sin(angle + arrowAngle * M_PI / 180.0f));

  // Arrowhead lines
  sf::Vertex arrowLine1[2];
  arrowLine1[0].position = pos + vec;
  arrowLine1[0].color = color;
  arrowLine1[1].position = pos + vec - arrowVec1;
  arrowLine1[1].color = color;

  sf::Vertex arrowLine2[2];
  arrowLine2[0].position = pos + vec;
  arrowLine2[0].color = color;
  arrowLine2[1].position = pos + vec - arrowVec2;
  arrowLine2[1].color = color;

  // Draw the arrowhead lines
  target.draw(arrowLine1, 2, sf::Lines);
  target.draw(arrowLine2, 2, sf::Lines);
}

#endif  // IMGUI_SFML_STARTER_DRAWING_H
