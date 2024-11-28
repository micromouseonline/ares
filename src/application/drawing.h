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

#endif  // IMGUI_SFML_STARTER_DRAWING_H
