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
      : window(sf::VideoMode(800, 600), "SFML + ImGui"),  //
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

  void renderLogs() {
    ImGui::Begin("Logs");
    for (const auto& log : logs) {
      ImGui::Text("%s", log.c_str());
    }
    ImGui::End();
  }

  void run() {
    sf::Clock deltaClock;
    bool scrollToBottom = true;
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

      ////////////// ImGui dialogue /////////////////////////////////////
      ImGui::Begin("Simulation model");
      for (int i = 15; i >= 0; i--) {
        DrawLEDx(target_pins[i], 6, IM_COL32(255, 64, 64, 255));
      }
      ImGui::Text("status LEDS");

      if (ImGui::Button("TOGGLE")) {
        manager.setPinState(11, LOW);
      }
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
      ImGui::Text("Sensor Data");
      ImGui::ProgressBar(static_cast<float>(target_sensors.lfs) / 255.0f, ImVec2(0.0f, 0.0f), "LFS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.lds) / 255.0f, ImVec2(0.0f, 0.0f), "LDS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.rds) / 255.0f, ImVec2(0.0f, 0.0f), "RDS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.rfs) / 255.0f, ImVec2(0.0f, 0.0f), "RFS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.battery) / 255.0f, ImVec2(0.0f, 0.0f), "BATT");

      ImGui::End();
      //////////////////////////////////////////////////////////////////
      ImGui::Begin("Simulation Controls");
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
      ImGui::End();
      //////////////////////////////////////////////////////////////////
      // Fetch logs from the Manager
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
      //////////////////////////////////////////////////////////////////
      renderLogs();

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
