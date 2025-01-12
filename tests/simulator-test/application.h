#include <imgui-SFML.h>
#include <imgui.h>
#include <SFML/Graphics.hpp>
#include <chrono>
#include <functional>
#include <thread>

#include "application//widgets.h"
#include "manager.h"
#include "target.h"

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

  void refreshUI(Manager &mgr) {
    mgr.updateLEDStates();
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

      ImGui::SFML::Update(window, deltaClock.restart());

      ImGui::Begin("Simulation model");
      target_pins = manager.getPins();
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
          ImGui::SameLine();
          ImGui::Text("Setting pin low\n");
          goButton = LOW;
          manager.setPinState(10, LOW);
        }
      } else {
        if (goButton == LOW) {
          ImGui::SameLine();
          ImGui::Text("Setting pin high\n");
          goButton = HIGH;
          manager.setPinState(10, HIGH);
        }
      }
      target_sensors = manager.getSensors();
      ImGui::Text("Sensor Data");
      ImGui::ProgressBar(static_cast<float>(target_sensors.lfs) / 255.0f, ImVec2(0.0f, 0.0f), "LFS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.lds) / 255.0f, ImVec2(0.0f, 0.0f), "LDS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.rds) / 255.0f, ImVec2(0.0f, 0.0f), "RDS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.rfs) / 255.0f, ImVec2(0.0f, 0.0f), "RFS");
      ImGui::ProgressBar(static_cast<float>(target_sensors.battery) / 255.0f, ImVec2(0.0f, 0.0f), "BATT");

      ImGui::End();

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
};
