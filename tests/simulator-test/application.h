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

  void ShowTargetLogWindow(const std::vector<std::string>& target_log) {
    static int start_index = 0;
    static bool scrolling = true;
    const int max_lines = 16;
    int log_size = target_log.size();
    const float line_height = ImGui::GetTextLineHeight();
    ImGui::SetNextWindowSizeConstraints(ImVec2(0, max_lines * line_height + 122),  //
                                        ImVec2(FLT_MAX, max_lines * line_height + 122));

    if (scrolling && log_size > max_lines) {
      start_index = log_size - 16;
    }
    if (start_index < 0) {
      start_index = 0;
    }
    int end_index = start_index + max_lines;
    if (end_index > log_size) {
      end_index = log_size;
    }
    ImGui::Begin("Target Log Window");
    // Display the portion of log_lines
    //    ImGui::BeginChild("ScrollingRegion", ImVec2(0, ImGui::GetTextLineHeight() * 16), true, ImGuiWindowFlags_HorizontalScrollbar);
    for (int i = start_index; i < end_index; ++i) {
      ImGui::TextUnformatted(target_log[i].c_str());
    }
    //    ImGui::EndChild();

    ImGui::Separator();
    const int button_width = 60;
    if (ImGui::Button("UP", ImVec2(button_width, 0)) && start_index > 0) {
      start_index -= max_lines;
      scrolling = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("DOWN", ImVec2(button_width, 0)) && end_index < log_size) {
      start_index += max_lines;
      scrolling = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("HOME", ImVec2(button_width, 0))) {
      scrolling = false;
      start_index = 0;
    }
    ImGui::SameLine();
    if (ImGui::Button("END", ImVec2(button_width, 0))) {
      scrolling = false;
      start_index = log_size - max_lines;
    }
    ImGui::SameLine();
    if (ImGui::Button(scrolling ? "PAUSE" : "RESUME", ImVec2(button_width, 0))) {
      scrolling = !scrolling;
    }
    ///// or use the mouse
    if (!scrolling && ImGui::IsWindowHovered()) {
      float wheel = ImGui::GetIO().MouseWheel;
      if (wheel > 0) {
        start_index -= 1;
      }
      if (wheel < 0) {
        start_index += 1;
      }
    }
    ImGui::End();
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
      target_pins = manager.getPins();
      target_sensors = manager.getSensors();
      int line_count = 1;
      manager.processOutput();
      ShowTargetLogWindow(manager.getLog());
      uint32_t ticks = manager.getTicks();
      int elapsed = ticks - last_ticks;
      last_ticks = ticks;

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
  SensorData target_sensors;
  std::array<bool, 16> target_pins;
  bool running = true;
  bool paused = false;
  LineProcessor line_processor;
  std::mutex manager_mutex;
};
