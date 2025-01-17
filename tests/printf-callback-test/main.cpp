
#include <imgui-SFML.h>
#include <imgui.h>
#include <SFML/Graphics.hpp>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <thread>

#include "application/line-processor.h"
#include "application/timer.h"
#include "common/printf/printf.h"

//////////////////////////////////////////////////////////////////////////////
/// Target class /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
/***
 * The Target class is designed to simulate embedded hardware behavior. It
 * does not rely on threading, mutexes, or atomic operations, ensuring
 * portability to environments with minimal runtime support.
 * Synchronization and thread management are handled externally by the
 * Manager class.
 */
class Target {
 public:
  bool is_running = true;
  volatile uint32_t ticks;

  using SerialCallback = std::function<void(const char)>;
  SerialCallback serialOut;

  Target() : ticks(0), serialOut(nullptr) {
  }

  ~Target() {
  }

  /// call this to terminate the thread
  void stopRunning() {
    is_running = false;
  }

  /***
   * Tell the target code how to output characters by calling an external
   * function
   *
   * @param cb
   */
  void setSerialoutCallback(SerialCallback cb) {
    serialOut = cb;
  }

  /***
   * On real hardware, systick would be triggered by a hardware timer
   * interrupt every millisecond. In this simulation, it is called
   * explicitly within delay_ms to simulate time passage. This approach
   * assumes delay_ms is called frequently to maintain accurate timing.
   * Note that the Timer class is non-blocking
   */
  void systick() {
    ticks += 1;
    Timer timer;
    timer.wait_us(1000);
  }

  /***
   * delay_ms simulates a blocking delay by repeatedly calling systick.
   * This design requires that delay_ms is called often enough to ensure
   * the simulated time progresses correctly.
   * @param ms
   */
  void delay_ms(uint32_t ms) {
    uint32_t end = ticks + ms;
    while (ticks < end) {
      systick();
    }
  }

  /***
   * This wrapper for the vsnprintf_ function will send its
   * output, character-by-character, to the given function.
   *
   * Call this just as you would normally call snprintf().
   *
   * The specific printf vsnprintf_ function here comes from the
   * Marco Paland lightweight printf library which uses no dynamic
   * memory and is thread safe.
   *
   * Note: Output is null-terminated to ensure compatibility with
   *       string processing functions and to mark the end of transmitted
   *       data explicitly.
   *
   * @return number of characters written including the termiator
   */
  int serialPrintf(Target::SerialCallback out, const char* format, ...) {
    if (!out) {
      return -1;  // Return error if no valid callback is provided
    }
    const int BUFFER_SIZE = 256;
    char buffer[BUFFER_SIZE];  // Adjust size as needed
    va_list args;
    va_start(args, format);
    /// remember to leave space for a terminating null
    int count = vsnprintf_(buffer, BUFFER_SIZE - 1, format, args);
    va_end(args);

    if (count > 0) {
      for (int i = 0; i < count && i < BUFFER_SIZE - 1; ++i) {
        out(buffer[i]);
      }
      out(buffer[count] = '\0');
      count++;
    }
    return count;  // Return the number of characters written
  }

  /// a convenience method that always prepends the tick count
  /// as a timestamp.
  void log(const char* message) {
    serialPrintf(serialOut, "%7u %s", ticks, message);
  }

  /***
   * The main loop should not return or the thread will terminate. A
   * different implementation might make the manager run the outer loop
   * but this way puts more control in the target code.
   */
  void mainLoop() {
    uint32_t interval = 1000;
    uint32_t last_update = ticks;
    while (is_running) {
      if ((ticks - last_update) >= interval) {
        log("update the blinker");
        last_update += interval;
      }
      delay_ms(1);
    }
  }
};

//////////////////////////////////////////////////////////////////////////////
/// Manager class ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
class Manager {
 public:
  Manager() : target(), output_queue(OUTPUT_QUEUE_SIZE), target_mutex(), log_mutex() {
    printf("Manager created\n");
    StartTarget();
  }

  /***
   * The Manager destructor must ensure that the target thread is stopped and joined.
   */
  ~Manager() {
    printf("Stopping the target\n");
    stopTarget();
    printf("Joining the target thread\n");
    if (target_thread.joinable()) {
      target_thread.join();
    }
    printf("Manager destroyed\n");
  }

  /***
   * The simulated target board runs in a separate thread and is started and
   * stopped by the manager. Before starting, the target is given the address
   * of the serial callback. while this could be done at any time, it makes
   * more sense to set things up before starting the target.
   */
  void StartTarget() {
    target.setSerialoutCallback([this](char c) { this->serialOutCallback(c); });
    printf("Start the target thread\n");
    target_thread = std::thread([this]() { target.mainLoop(); });
  }

  /***
   * When the application is done with the manager, the target thread must be
   * stopped. The thread function in the target is an endless loop which would
   * not normally return of its own accord. It is not a good idea to simply
   * kill the thread so we call the target's stopRunning method which will
   * set a flag telling the main loop to exit.
   *
   * Since the target thread is running when this method is called, it is
   * important to protect the method call with a mutex.
   */
  void stopTarget() {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.stopRunning();
  }

  /***
   * This method is called by the application. It extracts characters from
   * the output queue and adds them to a string. When the target emits a
   * string, it will ensure they are null terminated so it is possible to
   * embed CR and LF character if desired.
   * When the null is detected, the string is added to a vector of messages
   * and the string reset.
   * Note No checks are made that there is a null present.
   *      This may not be a good thing.
   */
  int processOutputQueue(std::vector<std::string>& log) {
    std::lock_guard<std::mutex> lock(log_mutex);
    int count = output_queue.size();
    static std::string s;
    while (!output_queue.empty()) {
      char c = output_queue.head();
      s += c;
      if (c == '\0') {
        log.push_back(s);
        s = "";
      }
    }
    return count;
  }

  /***
   * serialOutCallback simply places the characters in a queue much as you
   * might with a transmit buffer. In this demonstration code, the
   * queue is emptied on every display frame for display in the main
   * window.
   *
   * Several such callbacks might exist to simulate other devices.
   *
   * Note that the queue must be protected with a mutex to ensure
   *      the addition of characters does not interfere with the
   *      processing, by the application, of the contents of the queue.
   */
  void serialOutCallback(const char c) {
    std::lock_guard<std::mutex> lock(log_mutex);
    output_queue.push(c);
  }

  /***
   * The target maintains a tick count which we can access through a
   * getter function. For complete safety, we protect the call with
   * a mutex.
   */
  uint32_t getTicks() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.ticks;
  }

 private:
  Target target;
  std::thread target_thread;
  const int OUTPUT_QUEUE_SIZE = 2048;
  Queue<char> output_queue;  /// a serial output buffer for the target
  std::mutex target_mutex;   /// we need a mutex just the target
  std::mutex log_mutex;      /// and another for the logging queue
};

//////////////////////////////////////////////////////////////////////////////
/// Application class ////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/***
 * The application class simply creates a main window and some dialogues that can
 * be used for interaction with the target through the mediation of the manager.
 *
 * The manager is needed because the target code should be able to run without,
 * as far as possible, any knowledge of its environment. Ideally, the exact same
 * code would run on the actual target hardware and in the simulation
 *
 */
class Application {
 public:
  Application()
      : window(sf::VideoMode(1200, 800), WINDOW_TITLE),  //
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
        }
      }

      ImGui::SFML::Update(window, deltaClock.restart());

      int line_count = manager.processOutputQueue(logs);
      uint32_t ticks = manager.getTicks();
      int elapsed = ticks - last_ticks;
      last_ticks = ticks;

      ImGui::Begin("Simulation model");
      ImGui::Text("elapsed target ticks: %3d", elapsed);
      ImGui::Text("target log queue size: %3d", line_count);
      ImGui::End();

      ImGui::Begin("log");
      ImGui::Text("Log size: %d", (int)logs.size());
      for (auto& line : logs) {
        ImGui::Text("%s", line.c_str());
      }
      ImGui::End();

      window.clear();
      ImGui::SFML::Render(window);
      window.display();
    }
  }

 private:
  std::vector<std::string> logs;
  sf::RenderWindow window;
  Manager manager;
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
