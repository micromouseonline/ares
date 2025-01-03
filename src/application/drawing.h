//
// Created by peter on 22/11/24.
//

#ifndef DRAWING_H
#define DRAWING_H

#include <SFML/Graphics.hpp>
#include <cmath>

class Drawing {
 public:
  static sf::RectangleShape createRectangleShape(const sf::FloatRect& rect) {
    sf::RectangleShape rectangle;
    rectangle.setSize(sf::Vector2f(rect.width, rect.height));
    rectangle.setPosition(rect.left, rect.top);
    return rectangle;
  }

  static void drawFloatRect(sf::RenderTarget& target, sf::FloatRect& r, sf::Color colour = sf::Color::White) {
    sf::RectangleShape rectangleShape = createRectangleShape(r);
    rectangleShape.setFillColor(colour);
    target.draw(rectangleShape);
  }

  static void drawLine(sf::RenderTarget& target, sf::Vector2f start, sf::Vector2f end, sf::Color color = sf::Color::White) {
    sf::Vertex line[2];
    line[0].position = start;
    line[0].color = color;
    line[1].position = end;
    line[1].color = color;
    target.draw(line, 2, sf::Lines);
  }

  static void drawVector(sf::RenderTarget& target, sf::Vector2f pos, sf::Vector2f vec, sf::Color color = sf::Color::White) {
    sf::Vertex line[2];
    line[0].position = pos;
    line[0].color = color;
    line[1].position = pos + vec;
    line[1].color = color;
    target.draw(line, 2, sf::Lines);
  }

  static void drawDot(sf::RenderTarget& target, sf::Vector2f pos, float radius = 3, sf::Color colour = sf::Color(0, 255, 0, 192)) {
    sf::CircleShape c(radius);
    c.setOrigin(radius, radius);
    c.setFillColor(colour);
    c.setPosition(pos);
    target.draw(c);
  }

  // Function to draw a vector with an arrowhead
  static void drawVectorArrow(sf::RenderTarget& target, sf::Vector2f pos, sf::Vector2f vec, float arrowheadSizePercent, sf::Color color = sf::Color::White) {
    // Calculate the arrowhead size
    float vectorLength = std::sqrt(vec.x * vec.x + vec.y * vec.y);
    float arrowLength = vectorLength * (arrowheadSizePercent / 100.0f);  // Arrowhead length as a percentage of the vector length
    float arrowAngle = 30.0f * RADIANS;                                  // Angle between the arrowhead lines and the vector

    // Calculate the direction of the vector
    float angle = std::atan2(vec.y, vec.x);
    sf::Vector2f arrowVec1(arrowLength * std::cos(angle - arrowAngle), arrowLength * std::sin(angle - arrowAngle));
    sf::Vector2f arrowVec2(arrowLength * std::cos(angle + arrowAngle), arrowLength * std::sin(angle + arrowAngle));

    // Create a vertex array with 6 vertices: 2 for the main line, and 4 for the arrowhead
    sf::VertexArray vertices(sf::Lines, 6);

    // Main line
    vertices[0].position = pos;
    vertices[0].color = color;
    vertices[1].position = pos + vec;
    vertices[1].color = color;

    // Arrowhead lines
    vertices[2].position = pos + vec;
    vertices[2].color = color;
    vertices[3].position = pos + vec - arrowVec1;
    vertices[3].color = color;

    vertices[4].position = pos + vec;
    vertices[4].color = color;
    vertices[5].position = pos + vec - arrowVec2;
    vertices[5].color = color;

    // Draw the vertex array
    target.draw(vertices);
  }
};
#endif  // IMGUI_SFML_STARTER_DRAWING_H
