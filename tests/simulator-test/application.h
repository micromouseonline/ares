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

  void DisplayHexDump(const char* buffer, size_t size) {
    ImGui::Begin("Hex Dump");
    // Iterate through the buffer and display the hex values
    for (size_t i = 0; i < size; i += 16) {
      // Display address offset (hex)
      ImGui::Text("%04X ", i);
      ImGui::SameLine();
      // Display the hex values for the current line
      for (size_t j = 0; j < 16; ++j) {
        if (i + j < size) {
          ImGui::Text("%02X ", static_cast<unsigned char>(buffer[i + j]));
        } else {
          ImGui::Text("   ");  // Empty space for incomplete lines
        }
        ImGui::SameLine();
      }
      ImGui::SameLine();
      // Display the ASCII representation for the current line
      char txt[20];
      for (size_t j = 0; j < 16; ++j) {
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
    bool scrollToBottom = true;
    char logBuffer[LOG_BUFFER_SIZE] = {0};  // Buffer to hold log messages
    bool autoScroll = true;
    int log_size = 0;
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
      int log_space = manager.getLogRemaining();
      manager.getLogBuffer(logBuffer);

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
      ImGui::Text(std::to_string(log_space).c_str());
      ImGui::End();

      DisplayHexDump(logBuffer, LOG_BUFFER_SIZE);
      //////////////////////////////////////////////////////////////////
      // Fetch logs from the Manager
      /*
      auto logs = manager.getLogs();
      if (logs.size() > log_size) {
        scrollToBottom = true;
        log_size = logs.size();
      } else {
        scrollToBottom = false;
      }
      // ImGui window for logs
      ImGui::Begin("Log Window");

      // Display logs
      for (const auto& log : logs) {
        ImGui::TextUnformatted(log.c_str());
      }
      //////////////////////////////////////////////////////////////////

      // Check if the user is at the bottom of the log window
      float scrollY = ImGui::GetScrollY();
      float maxScrollY = ImGui::GetScrollMaxY();
      //      if (scrollY >= maxScrollY) {
      //        scrollToBottom = true;  // User is at the bottom, so we can scroll to bottom automatically
      //      } else {
      //        scrollToBottom = false;  // User is not at the bottom, manual scroll is happening
      //      }

      // Scroll to the bottom if necessary
      if (scrollToBottom) {
        ImGui::SetScrollHereY(1.0f);  // Scroll to the bottom
      }

      ImGui::End();
       */
      //////////////////////////////////////////////////////////////////
      ImGui::Begin("Target Log");
      ImGui::Checkbox("Auto-scroll", &autoScroll);
      ImGui::TextWrapped("%s", logBuffer);
      if (autoScroll) {
        ImGui::SetScrollHereY(1.0f);  // Scroll to the bottom
      }
      ImGui::End();
      //////////////////////////////////////////////////////////////////
      //      renderLogs();
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
  bool running = true;
  bool paused = false;
};
