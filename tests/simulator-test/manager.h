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
  std::thread manager_loop_thread;
  std::thread target_thread;
  std::atomic<bool> paused;
  std::mutex target_mutex;
  std::condition_variable pauseCV;
  Queue<char> target_serial_out;
  Target target;
  std::vector<std::string> target_log;
  LineProcessor processor;

 public:
  Manager() : target_serial_out(LOG_BUFFER_SIZE), target(), target_mutex(), processor(&target_mutex) {
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

  Target& getTarget() {
    return target;
  }

  void RunTarget() {
    target.setLogCallback([this](const char* message) { this->logCallback(message); });
    printf("Start the target thread\n");
    target_thread = std::thread([this]() { target.mainLoop(); });
  }
  ////////////////////////////////////
  void stopTarget() {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.stopRunning();
  }

  void pauseTarget() {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.pauseRunning();
  }

  void resumeTarget() {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.resumeRunning();
  }
  ////////////////////////////////////

  void setParameter(float p) {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.setFilterAlpha(p);
  }

  float getParameter() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.getFilterAlpha();
  }

  int processOutput() {
    //    std::lock_guard<std::mutex> lock(target_mutex);
    int count = processor.processQueue(target_serial_out, target_log);
    return count;
  }

  void logCallback(const char* msg) {
    std::lock_guard<std::mutex> lock(target_mutex);
    const char* c = msg;
    while (*c) {
      target_serial_out.push(*c);
      c++;
    }
    target_serial_out.push('\n');
    target_serial_out.push('\0');
  }

  const std::vector<std::string>& getLog() {
    return target_log;
  }

  uint32_t getTicks() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.ticks;
  }

  void setPinState(int i, bool state) {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.digitalWrite(i, state);
  }

  std::array<bool, 16> getPins() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.getPinState();
  }

  SensorData getSensors() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.getSensors();
  }

  void setSensorCallback(SensorCallbackFunction cb) {
    target.setSensorCallback(cb);
  }
};
