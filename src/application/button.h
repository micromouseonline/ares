//
// Created by peter on 13/11/24.
//

#ifndef IMGUI_SFML_STARTER_BUTTON_H
#define IMGUI_SFML_STARTER_BUTTON_H

#include <SFML/Graphics.hpp>
// Button Event Enum
enum class ButtonEvent {
  None,     // No event
  Hovered,  // Mouse hovering over the button
  Pressed,  // Button pressed
  Released  // Button released (action triggered)
};

// Button class
class Button {
 public:
  Button(float x, float y, float width, float height, const sf::String& text, const sf::Font& font) : rect({width, height}), isPressed(false) {
    normalColor = sf::Color(0, 88, 0);
    hoverColor = sf::Color(0, 68, 0);
    pressedColor = sf::Color(0, 98, 0);
    rect.setOrigin(rect.getSize().x / 2, rect.getSize().y / 2);
    rect.setPosition(x, y);
    rect.setOutlineThickness(2.0);
    rect.setOutlineColor(normalColor + sf::Color(40, 40, 40));
    rect.setFillColor(normalColor);
    label.setFont(font);
    label.setCharacterSize(20);
    label.setFillColor(sf::Color::White);
    set_text(text);
  }

  void draw(sf::RenderWindow& window) {
    window.draw(rect);
    window.draw(label);
  }

  void set_text(const sf::String& text) {
    label.setString(text);
    float x = rect.getPosition().x;
    float y = rect.getPosition().y;

    float h = label.getGlobalBounds().height;
    float w = label.getGlobalBounds().width;
    label.setPosition(x - w / 2, y - h / 2 - 5);
  }

  ButtonEvent handleEvent(const sf::Event& event, const sf::RenderWindow& window) {
    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
    bool isHovered = rect.getGlobalBounds().contains(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));

    if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left && isHovered) {
      isPressed = true;
      rect.setFillColor(pressedColor);
      return ButtonEvent::Pressed;
    }

    if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
      if (isHovered && isPressed) {
        isPressed = false;
        rect.setFillColor(normalColor);
        return ButtonEvent::Released;
      }
      isPressed = false;
      rect.setFillColor(normalColor);
    }

    if (!isPressed && isHovered) {
      rect.setFillColor(hoverColor);
      return ButtonEvent::Hovered;
    }

    if (!isPressed) {
      rect.setFillColor(normalColor);
    }

    return ButtonEvent::None;
  }

 private:
  sf::RectangleShape rect;
  sf::Text label;
  sf::Color normalColor;
  sf::Color hoverColor;
  sf::Color pressedColor;
  bool isPressed;
};
#endif  // IMGUI_SFML_STARTER_BUTTON_H
