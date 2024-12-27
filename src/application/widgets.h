//
// Created by peter on 27/12/24.
//

#pragma once

#include <imgui-SFML.h>
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

// Custom widget function
inline bool CustomButton(const char* label, const ImVec2& size = ImVec2(0, 0)) {
  ImGui::Button(label, size);  // Draw the button
  ImGuiIO& io = ImGui::GetIO();
  ImVec2 mouse_pos = io.MousePos;
  ImVec2 button_min = ImGui::GetItemRectMin();
  ImVec2 button_max = ImGui::GetItemRectMax();

  // Check if mouse is inside the button and button is down
  bool is_inside = (mouse_pos.x >= button_min.x && mouse_pos.x <= button_max.x && mouse_pos.y >= button_min.y && mouse_pos.y <= button_max.y);
  bool is_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);

  return is_inside && is_down;
}
