#pragma once

#ifdef _WIN32
#include <windows.h>
#else
#include <chrono>
#include <thread>
#endif

class Timer {
 public:
  Timer() {
#ifdef _WIN32
    QueryPerformanceFrequency(&frequency);
#endif
  }

  void start() {
#ifdef _WIN32
    QueryPerformanceCounter(&start_time);
#else
    start_time = std::chrono::steady_clock::now();
#endif
  };

  // Note: this is used in a busy-wait that fully uses one core
  bool hasElapsed(uint32_t microseconds) {
#ifdef _WIN32
    QueryPerformanceCounter(&end_time);
    LONGLONG elapsed = end_time.QuadPart - start_time.QuadPart;
    return elapsed >= frequency.QuadPart * microseconds / 1000000;
#else
    auto current_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
    return duration >= microseconds;
#endif
  };

  void wait_us(uint32_t microseconds) {
#ifdef _WIN32
    start();
    while (!hasElapsed(microseconds)) {
      // busy-wait
    }
#else
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
#endif
  }

  void wait_ms(uint32_t milliseconds) {
    uint32_t microseconds = 1000UL * milliseconds;
#ifdef _WIN32
    start();
    while (!hasElapsed(microseconds)) {
      // busy-wait
    }
#else
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
#endif
  }

 private:
#ifdef _WIN32
  LARGE_INTEGER frequency;
  LARGE_INTEGER start_time;
  LARGE_INTEGER end_time;
#else
  std::chrono::steady_clock::time_point start_time;
#endif
};
