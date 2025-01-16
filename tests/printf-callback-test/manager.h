//
// Created by peter on 12/01/25.
//

#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "commands.h"
#include "common/line-processor.h"
#include "common/timer.h"
#include "target.h"

// Manager class
class Manager {
 public:
  const int OUTPUT_QUEUE_SIZE = 2048;
  Target target;
  std::thread target_thread;
  Queue<char> target_serial_out;
  /// and a mutex just for the logging queue
  std::mutex target_mutex;
  std::mutex log_mutex;
  std::condition_variable pauseCV;
  std::vector<std::string> target_log;

 public:
  Manager() : target(), target_serial_out(OUTPUT_QUEUE_SIZE), target_mutex(), log_mutex() {
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

  int processOutput() {
    std::lock_guard<std::mutex> lock(log_mutex);
    int count = target_serial_out.size();
    while (!target_serial_out.empty()) {
      std::cout << target_serial_out.head();
    }
    return count;
  }

  void serialOutCallback(const char c) {
    std::lock_guard<std::mutex> lock(log_mutex);
    target_serial_out.push(c);
  }

  const std::vector<std::string>& getLog() {
    return target_log;
  }

  uint32_t getTicks() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.ticks;
  }
};
