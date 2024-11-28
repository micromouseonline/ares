//
// Created by peter on 22/11/24.
//

#ifndef TEXTBOX_H
#define TEXTBOX_H

#include <SFML/Graphics.hpp>
#include <string>

/***
 * The Textbox class is a simple text overlay that displays the most recent messages
 * in a scrolling window.
 *
 * Set the width of the window. The height will adjust according to the number of messages
 * and the font size.
 *
 * Only the most recent messages will be displayed.
 *
 */
class Textbox {
 public:
  Textbox() {
    Setup(5, 9, 200, sf::Vector2f(0, 0));  //
  }

  Textbox(int l_visible, int l_charSize, int l_width, sf::Vector2f l_screenPos) {
    Setup(l_visible, l_charSize, l_width, l_screenPos);  //
  }

  ~Textbox() {
    Clear();  //
  };

  void Setup(int lines, int charSize, int width, sf::Vector2f position) {
    m_visible_lines = lines;
    m_font.loadFromFile("assets/fonts/FreeMono.ttf");
    m_text.setFont(m_font);
    m_text.setString("TextBox Here");
    m_text.setCharacterSize(charSize);
    sf::Vector2f padding(8.0f, 2.0f);  // padding at top left
    m_text.setPosition(position + padding);

    m_background.setSize(sf::Vector2f(width + 2 * padding.x, (lines * (charSize * 1.2f) + 2 * padding.y)));
    m_background.setPosition(position);

    SetTextColor(sf::Color::White);
    SetBackgroundColor(sf::Color(90, 90, 90, 90));
    SetBorderColor(sf::Color(90, 90, 0, 190));
  }

  void SetTextColor(sf::Color color) {
    m_text.setFillColor(color);  //
  }

  void SetBackgroundColor(sf::Color color) {
    m_background.setFillColor(color);  //
  }

  void SetBorderColor(sf::Color color) {
    m_background.setOutlineColor(color);
    m_background.setOutlineThickness(1.0f);
  }

  void Add(std::string str) {
    m_messages.push_back(str);
    if (m_messages.size() <= m_visible_lines) {
      return;
    }
    m_messages.erase(m_messages.begin());
  }

  void Clear() {
    m_messages.clear();  //
  };

  void Render(sf::RenderWindow& l_wind) {
    std::string content;
    for (auto& itr : m_messages) {
      content.append(itr + "\n");
    }
    if (content != "") {
      m_text.setString(content);
      l_wind.draw(m_background);
      l_wind.draw(m_text);
    }
  }

 private:
  std::vector<std::string> m_messages;
  size_t m_visible_lines;
  sf::RectangleShape m_background;
  sf::Font m_font;
  sf::Text m_text;
};

#endif  // TEXTBOX_H
