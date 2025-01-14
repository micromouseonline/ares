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
    ImGui::Begin("Mouse Events");
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
inline bool PushButton(const char* label, const ImVec2& size = ImVec2(0, 0)) {
  ImGui::Button(label, size);  // Draw the button
  bool mouseDown = ImGui::IsMouseDown(ImGuiMouseButton_Left);
  bool hovered = ImGui::IsItemHovered();
  return hovered && mouseDown;
}

// Function to draw an LED indicator
inline void DrawLED(bool state, const ImVec4& color) {
  ImGui::PushStyleColor(ImGuiCol_Button, state ? color : ImVec4(0.2f, 0.2f, 0.2f, 1.0f));
  ImGui::Button("##LED", ImVec2(20, 20));
  ImGui::PopStyleColor();
}

ImU32 dimColour(ImU32 color) {
  ImVec4 col = ImGui::ColorConvertU32ToFloat4(color);
  //  col.x *= 0.25;
  //  col.y *= 0.25;
  //  col.z *= 0.25;
  col.w *= 0.25;
  return ImGui::ColorConvertFloat4ToU32(col);
}
void DrawLEDx(bool value, float radius, ImU32 col = IM_COL32(255, 0, 0, 255)) {
  float lineHeight = ImGui::GetTextLineHeight();
  ImVec2 pos = ImGui::GetCursorScreenPos();
  pos.x = pos.x + radius;
  pos.y = pos.y + lineHeight / 2 + 1;
  ImU32 color = value ? col : dimColour(col);
  ImGui::GetWindowDrawList()->AddCircleFilled(pos, radius, color);
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 2 * radius + 5);
}

inline void drawSensorUpdateTime(int sensor_update_time) {
  static int update_time = sensor_update_time;
  float alpha = 0.025;
  update_time = alpha * (sensor_update_time) + (1 - alpha) * update_time;
  static int peak_time = 0;
  if (sensor_update_time > peak_time) {
    peak_time = sensor_update_time;
  } else {
    peak_time = 0.98 * peak_time;
  }
  ImGui::NewLine();
  ImGui::Text("Sensor update %3d us", update_time);
  ImGui::SameLine();
  ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::GetWindowDrawList()->AddRect(p, ImVec2(p.x + 200, p.y + 20), IM_COL32(255, 200, 0, 255));
  ImColor bar_color = IM_COL32(0, 255, 0, 128);
  p.x += 1;
  p.y += 1;
  ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + std::min(198, peak_time), p.y + 18), IM_COL32(200, 0, 0, 128));
  ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + update_time, p.y + 18), bar_color);
  ImGui::NewLine();
}

void DisplayHexDump(const char* buffer, int size) {
  ImGui::Begin("Hex Dump");
  // Iterate through the buffer and display the hex values
  for (int i = 0; i < size; i += 16) {
    // Display address offset (hex)
    ImGui::Text("%04X ", i);
    ImGui::SameLine();
    // Display the hex values for the current line
    for (int j = 0; j < 16; ++j) {
      if (i + j < size) {
        ImGui::Text("%02X ", static_cast<unsigned char>(buffer[i + j]));
      } else {
        ImGui::Text("   ");  // Empty space for incomplete lines
      }
      ImGui::SameLine();
    }
    ImGui::SameLine();
    // Display the ASCII representation for the current line
    for (int j = 0; j < 16; ++j) {
      if (i + j < size) {
        char c = buffer[i + j];
        ImGui::Text("%c", (c >= 32 && c <= 126) ? c : '.');
      } else {
        ImGui::Text(" ");  // Empty space for incomplete lines
      }
      ImGui::SameLine();
    }
    ImGui::NewLine();
  }
  ImGui::End();
}
