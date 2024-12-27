//
// Created by peter on 22/11/24.
//

#ifndef TEXTBOX_H
#define TEXTBOX_H

#include <imgui.h>
#include <SFML/Graphics.hpp>
#include <string>

class TextBox {
 public:
  TextBox() : m_scroll_to_bottom(true) {};
  ~TextBox() {};

  void addText(const std::string& text) {
    lines.push_back(text);
    m_scroll_to_bottom = true;
  }

  void clear() {
    lines.clear();
  }

  void render() {
    //    m_sfml_textbox.initialise(10, 14, 400, sf::Vector2f(1000, 10));
    ImGui::Begin("TEXT BOX");
    for (const auto& line : lines) {
      ImGui::TextWrapped("%s", line.c_str());
    }
    if (m_scroll_to_bottom) {
      ImGui::SetScrollHereY(1.0f);
      m_scroll_to_bottom = false;
    }
    ImGui::End();
  }

 private:
  std::vector<std::string> lines;
  bool m_scroll_to_bottom;
};
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
class sfmlTextbox {
 public:
  sfmlTextbox() {
    initialise(5, 9, 200, sf::Vector2f(conf::WindowPadding, 8 * conf::WindowPadding));  //
  }

  sfmlTextbox(int l_visible, int l_charSize, int l_width, sf::Vector2f l_screenPos) {
    initialise(l_visible, l_charSize, l_width, l_screenPos);  //
  }

  ~sfmlTextbox() {
    clear();  //
  };

  void initialise(int lines, int charSize, int width, sf::Vector2f position) {
    m_visible_lines = lines;
    m_font.loadFromFile("assets/fonts/ubuntu-mono-r.ttf");
    m_text.setFont(m_font);
    m_text.setString("TextBox Here");
    m_text.setCharacterSize(charSize);
    sf::Vector2f padding(10.0f, 10.0f);  // padding at top left
    m_text.setPosition(position + padding);

    m_background.setSize(sf::Vector2f(width + 2 * padding.x, (lines * (charSize * 1.2f) + 2 * padding.y)));
    m_background.setPosition(position);

    setTextColour(sf::Color::White);
    setBackgroundColour(sf::Color(90, 90, 90, 90));
    setBorderColour(sf::Color(90, 90, 0, 190));
  }

  void setTextColour(sf::Color color) {
    m_text.setFillColor(color);  //
  }

  void setBackgroundColour(sf::Color color) {
    m_background.setFillColor(color);  //
  }

  void setBorderColour(sf::Color color) {
    m_background.setOutlineColor(color);
    m_background.setOutlineThickness(1.0f);
  }

  void addString(std::string str) {
    m_messages.push_back(str);
    if (m_messages.size() <= m_visible_lines) {
      return;
    }
    m_messages.erase(m_messages.begin());
  }

  void clear() {
    m_messages.clear();  //
  };

  void draw(sf::RenderWindow& l_wind) {
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
