#include <imgui-SFML.h>
#include <imgui.h>
#include <SFML/Graphics.hpp>
#include <chrono>
#include <functional>
#include <thread>

#include "application/widgets.h"
#include "common/line-processor.h"
#include "manager.h"
#include "target.h"

std::vector<std::string> logs;

class Application {
 public:
  Application()
      : window(sf::VideoMode(1200, 800), "SFML + ImGui"),  //
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
  }

  void run() {
    sf::Clock deltaClock;
    while (window.isOpen()) {
      static uint32_t last_ticks = manager.getTicks();
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
      int line_count = manager.processOutput();
      uint32_t ticks = manager.getTicks();
      int elapsed = ticks - last_ticks;
      last_ticks = ticks;

      ////////////// ImGui dialogue /////////////////////////////////////
      ImGui::Begin("Simulation model");

      static bool goButton = true;
      if (ImGui::Button(paused ? "Resume" : "Pause")) {
        paused = !paused;
      }

      ImGui::SeparatorText("Sensor Data");
      ImGui::Text("elapsed target ticks: %3d", elapsed);
      ImGui::Text("target log queue size: %3d", line_count);
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
  bool running = true;
  bool paused = false;
};
