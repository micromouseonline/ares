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
    static SensorData last_known_data = target_sensors;
    if (paused) {
      return last_known_data;
    }
    static int q = 0;
    q = (q + 1) % 2550;
    SensorData data = target_sensors;
    data.lfs = target_pins[pin] * 100;
    data.lds = q / 10;
    data.rds = (data.rds + 1) % 128;
    last_known_data = data;
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

      if (ImGui::Button("RESET")) {
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
          manager.pauseCV.notify_all();  // Notify Manager to pause
        } else {
          manager.pauseCV.notify_all();  // Notify Manager to resume
        }
      }
      ImGui::End();
      ImGui::Begin("Simulation Status");
      if (paused) {
        ImGui::Text("Simulation Paused");
      } else {
        ImGui::Text("Simulation Running");
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
