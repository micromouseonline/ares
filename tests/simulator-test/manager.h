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

#include "commands.h"
#include "common/timer.h"
#include "target.h"

// Manager class
class Manager {
 public:
  Target target;
  std::thread manager_loop_thread;
  std::thread target_main_thread;
  std::atomic<bool> manager_running;
  std::atomic<bool> paused;
  std::mutex target_mutex;
  std::condition_variable pauseCV;
  Queue<char> target_serial_out;

 public:
  Manager() : manager_running(true), target_serial_out(LOG_BUFFER_SIZE) {
    manager_loop_thread = std::thread(&Manager::RunTarget, this);
    printf("Manager created\n");
  }

  ~Manager() {
    manager_running = false;
    printf("Make sure the command queue is processed\n");
    //    commandCV.notify_all();
    printf("Stop the target main loop\n");
    stopTarget();
    printf("Join the Target main thread\n");
    if (target_main_thread.joinable()) {
      target_main_thread.join();
    }
    printf("Join the Manager loop thread\n");
    if (manager_loop_thread.joinable()) {
      manager_loop_thread.join();
    }
    printf("Manager destroyed\n");
  }

  Target& getTarget() {
    return target;
  }

  void RunTarget() {
    /// We can freely access the target here because
    /// its thread is not running
    target.setup();

    /// start the thread that the target runs in. When the manager has
    /// finished with the target, be sure to call the target's
    /// stopRunning method.
    target_main_thread = std::thread([this]() { target.mainLoop(); });
    while (manager_running) {
      /// nothing to see here but we could be feeding stuff to the target
      while (!target.output_buffer.empty()) {
        std::lock_guard<std::mutex> lock(target_mutex);
        char c = target.output_buffer.head();
        target_serial_out.push(c);
      }
      Timer timer;
      timer.wait_us(3000);
    }
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

  int getLogBuffer() {
    std::lock_guard<std::mutex> lock(target_mutex);
    int result = 0;
    while (!target.output_buffer.empty()) {
      result++;
      char c = target.output_buffer.head();
      target_serial_out.push(c);
    }
    return result;
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
