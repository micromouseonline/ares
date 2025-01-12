#include <imgui-SFML.h>
#include <imgui.h>
#include <SFML/Graphics.hpp>
#include <chrono>
#include <functional>
#include <thread>

#include "manager.h"
#include "target.h"

class Application {
 public:
  Application()
      : window(sf::VideoMode(800, 600), "SFML + ImGui"),  //
        manager(),
        uiThread(&Application::uiThreadFunction, this) {
    ImGui::SFML::Init(window);
    setup();
  }

  ~Application() {
    if (uiThread.joinable()) {
      uiThread.join();
    }
    ImGui::SFML::Shutdown();
  }

  void setup() {
    manager.setSensorCallback([this](int pin) { return this->sensorCallback(pin); });
  }

  void uiThreadFunction() {
    while (running) {
      onButtonPress(10, manager);
      refreshUI(manager);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  SensorData sensorCallback(int pin) {
    SensorData data;
    data.lfs = pin;
    return data;
  }

  void onButtonPress(int buttonID, Manager &mgr) {
    mgr.simulateButtonPress(buttonID);
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

      ImGui::Begin("Hello, ImGui!");
      ImGui::Text("This is an ImGui window.");
      ImGui::End();

      window.clear();
      ImGui::SFML::Render(window);
      window.display();
    }
  }

 private:
  sf::RenderWindow window;
  Manager manager;
  std::thread uiThread;
  bool running = true;
};
