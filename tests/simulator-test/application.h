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
    SensorData data;
    data.lfs = pin;
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
      ImGui::Text("Target LEDs");

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

      ImGui::End();

      window.clear();
      ImGui::SFML::Render(window);
      window.display();
    }
  }

 private:
  sf::RenderWindow window;
  Manager manager;
  //  std::thread uiThread;
  bool running = true;
};
