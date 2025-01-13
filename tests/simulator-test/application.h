#include <imgui-SFML.h>
#include <imgui.h>
#include <SFML/Graphics.hpp>
#include <chrono>
#include <functional>
#include <thread>

#include "application//widgets.h"
#include "manager.h"
#include "target.h"

std::vector<std::string> logs;

class Application {
 public:
  Application()
      : window(sf::VideoMode(1200, 600), "SFML + ImGui"),  //
        manager() {
    window.setVerticalSyncEnabled(true);
    if (!ImGui::SFML::Init(window)) {
      exit(1);
    };
    setup();
    logMessage("Good to go");
  }

  ~Application() {
    ImGui::SFML::Shutdown();
  }

  void setup() {
    manager.setSensorCallback([this](int pin) { return this->sensorCallback(pin); });
  }

  SensorData sensorCallback(int pin) {
    static int q = 0;
    q = (q + 1) % 2550;
    SensorData data = target_sensors;
    data.lfs = target_pins[pin] * 100;
    data.lds = q / 10;
    data.rds = (data.rds + 1) % 128;
    return data;
  }

  void logMessage(const std::string& message) {
    logs.push_back(message);
    if (logs.size() > 100) {
      logs.erase(logs.begin());
    }
  }

  int addLinesToTargetLog(const char* buffer, size_t size) {
    int count = 0;     // Count of lines added
    size_t start = 0;  // Start index of the current line

    while (start < size && buffer[start] != '\0') {
      size_t end = start;

      // Find the end of the current line
      while (end < size && buffer[end] != '\n' && buffer[end] != '\0') {
        end++;
      }

      // If a line is found, add it to the log
      if (end > start) {
        target_log.emplace_back(buffer + start, end - start);
        count++;  // Increment line count
      }

      // Skip the newline character if present
      if (end < size && buffer[end] == '\n') {
        end++;
      }

      // Move start to the end for the next iteration
      start = ++end;
    }

    return count;
  }

  void displayTargetLog() {
    ImGui::Begin("Target Log Window");
    // Make the window scrollable
    ImGui::BeginChild("ScrollingRegion", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    // Display each line
    for (const auto& line : target_log) {
      ImGui::TextUnformatted(line.c_str());
    }
    // Scroll to the bottom if at the bottom already
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
      ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();
    ImGui::End();
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

  void run() {
    sf::Clock deltaClock;
    char logBuffer[LOG_BUFFER_SIZE] = {0};  // Buffer to hold log messages
    while (window.isOpen()) {
      sf::Event event;
      while (window.pollEvent(event)) {
        ImGui::SFML::ProcessEvent(window, event);
        if (event.type == sf::Event::Closed) {
          window.close();
          running = false;
        }
      }

      /// update with window state and events. Prepare for rendering
      ImGui::SFML::Update(window, deltaClock.restart());

      ////////////// update Local copies of the target state ////////////
      target_pins = manager.getPins();
      target_sensors = manager.getSensors();
      static int log_space_used = 0;
      if (manager.getLogBuffer(logBuffer)) {
        log_space_used = addLinesToTargetLog(logBuffer, LOG_BUFFER_SIZE);
      }
      DisplayHexDump(logBuffer, LOG_BUFFER_SIZE);
      displayTargetLog();

      ////////////// ImGui dialogue /////////////////////////////////////
      ImGui::Begin("Simulation model");
      ImGui::SeparatorText("IO Pins");
      for (int i = 15; i >= 0; i--) {
        DrawLEDx(target_pins[i], 6, IM_COL32(255, 64, 64, 255));
      }
      ImGui::NewLine();
      ImGui::SeparatorText("Actions");
      if (ImGui::Button("TOGGLE")) {
        manager.setPinState(11, LOW);
      }
      ImGui::SameLine();
      static bool goButton = HIGH;
      if (PushButton("GO")) {
        if (goButton == HIGH) {
          goButton = LOW;
          manager.setPinState(10, LOW);
        }
      } else {
        if (goButton == LOW) {
          goButton = HIGH;
          manager.setPinState(10, HIGH);
        }
      }
      ImGui::SameLine();
      if (ImGui::Button(paused ? "Resume" : "Pause")) {
        paused = !paused;
        if (paused) {
          manager.pauseTarget();
          manager.pauseCV.notify_all();  // Notify Manager to pause
        } else {
          manager.resumeTarget();
          manager.pauseCV.notify_all();  // Notify Manager to resume
        }
      }
      ImGui::SeparatorText("Parameters");
      float alpha = manager.getParameter();
      if (ImGui::SliderFloat("Filter Alpha", &alpha, 0.0f, 1.0f)) {
        // When the slider value changes, update the target parameter
        manager.setParameter(alpha);
      }
      ImGui::SeparatorText("Sensor Data");
      ImGui::ProgressBar(static_cast<float>(target_sensors.lfs) / 255.0f, ImVec2(0.0f, 0.0f), "LFS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.lds) / 255.0f, ImVec2(0.0f, 0.0f), "LDS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.rds) / 255.0f, ImVec2(0.0f, 0.0f), "RDS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.rfs) / 255.0f, ImVec2(0.0f, 0.0f), "RFS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.battery) / 255.0f, ImVec2(0.0f, 0.0f), "BATT");
      ImGui::Text("Target Log lines: %d (%d)", (int)target_log.size(), log_space_used);
      ImGui::ProgressBar(static_cast<float>(log_space_used) / LOG_BUFFER_SIZE, ImVec2(0.0f, 0.0f), "% free");
      ImGui::End();

      //////////////////////////////////////////////////////////////////
      window.clear();
      ImGui::SFML::Render(window);
      window.display();
    }
  }

 private:
  sf::RenderWindow window;
  Manager manager;
  SensorData target_sensors;
  std::array<bool, 16> target_pins;
  std::vector<std::string> target_log;
  bool running = true;
  bool paused = false;
};
