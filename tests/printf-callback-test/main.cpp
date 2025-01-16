
#include <iostream>
#include <thread>

#include <imgui-SFML.h>
#include <imgui.h>
#include <SFML/Graphics.hpp>
#include <condition_variable>
#include <functional>

#include "application/widgets.h"
#include "common/line-processor.h"
#include "common/printf/printf.h"
#include "common/timer.h"

std::vector<std::string> logs;

// Simulated Arduino Nano Target class
/// Target class /////////////////////////////////////////////////////////////////
class Target {
 public:
  bool is_running = true;
  volatile uint32_t ticks;

  using SerialCallback = std::function<void(const char)>;
  SerialCallback serialOut;

  Target() : ticks(0), serialOut(nullptr) {
    printf("Target setup\n");
  }

  ~Target() {
    printf("Target cleanup\n");
  }

  void setSerialoutCallback(SerialCallback cb) {
    serialOut = cb;
  }

  void timerISR() {
    ticks += 1;
    Timer timer;
    timer.wait_us(1000);
  }

  void delay_ms(uint32_t ms) {
    uint32_t end = ticks + ms;
    while (ticks < end) {
      timerISR();
    }
  }

  void log(const char* message) {
    char buf[128];
    int count = snprintf(buf, 126, "%7u %s", ticks, message);
    for (int i = 0; i < count; i++) {
      serialOut(buf[i]);
    }
    std::cout << buf;
    return;
  }

  void stopRunning() {
    is_running = false;
  }

  void mainLoop() {
    uint32_t interval = 1000;
    uint32_t next_update = ticks + interval;
    while (is_running) {
      if (ticks >= next_update) {
        log("update the blinker\n");
        next_update += interval;
      }
      delay_ms(1);
    }
    delay_ms(1);  // get at least one tick in every cycle
  }
};

/// Manager class /////////////////////////////////////////////////////////////////
class Manager {
 public:
  const int OUTPUT_QUEUE_SIZE = 2048;
  Target target;
  std::thread target_thread;
  Queue<char> output_queue;
  /// and a mutex just for the logging queue
  std::mutex target_mutex;
  std::mutex log_mutex;
  std::condition_variable pauseCV;

 public:
  Manager() : target(), output_queue(OUTPUT_QUEUE_SIZE), target_mutex(), log_mutex() {
    printf("Manager created\n");
    RunTarget();
  }

  ~Manager() {
    printf("Stop the target\n");
    stopTarget();
    printf("Join the target thread\n");
    if (target_thread.joinable()) {
      target_thread.join();
    }
    printf("Manager destroyed\n");
  }

  void RunTarget() {
    target.setSerialoutCallback([this](char c) { this->serialOutCallback(c); });
    printf("Start the target thread\n");
    target_thread = std::thread([this]() { target.mainLoop(); });
  }
  ////////////////////////////////////
  void stopTarget() {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.stopRunning();
  }

  int processOutput(std::vector<std::string>& log) {
    std::lock_guard<std::mutex> lock(log_mutex);
    int count = output_queue.size();
    static std::string s;
    while (!output_queue.empty()) {
      char c = output_queue.head();
      s += c;
      if (c == '\n') {
        log.push_back(s);
        s = "";
      }
    }
    return count;
  }

  void serialOutCallback(const char c) {
    std::lock_guard<std::mutex> lock(log_mutex);
    output_queue.push(c);
  }

  uint32_t getTicks() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.ticks;
  }
};

/// Application class /////////////////////////////////////////////////////////////////
class Application {
 public:
  Application()
      : window(sf::VideoMode(1200, 800), "SFML + ImGui"),  //
        manager() {
    window.setVerticalSyncEnabled(true);
    if (!ImGui::SFML::Init(window)) {
      exit(1);
    };
  }

  ~Application() {
    ImGui::SFML::Shutdown();
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
      int line_count = manager.processOutput(logs);
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

      ImGui::Begin("log");
      ImGui::Text("Log size: %d", (int)logs.size());
      for (auto& line : logs) {
        ImGui::Text("%s", line.c_str());
      }
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

extern "C" void _putchar(char c) {
  std::cout << c;
}
// Main function
int main() {
  Application app;
  app.run();
  return 0;
}
