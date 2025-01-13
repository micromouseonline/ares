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
#include "target.h"

// Manager class
class Manager {
  Target target;
  std::thread manager_loop_thread;
  std::thread target_main_thread;
  std::atomic<bool> manager_running;
  std::mutex target_mutex;
  std::queue<Command> commandQueue;
  std::condition_variable commandCV;

 public:
  Manager() : manager_running(true) {
    manager_loop_thread = std::thread(&Manager::RunTarget, this);
    printf("Manager created\n");
  }

  ~Manager() {
    manager_running = false;
    printf("Make sure the command queue is processed\n");
    commandCV.notify_all();
    printf("Stop the target main loop\n");
    stopTarget();
    printf("Join the Manager loop thread\n");
    if (manager_loop_thread.joinable()) {
      manager_loop_thread.join();
    }
    printf("Join the Target main thread\n");
    if (target_main_thread.joinable()) {
      target_main_thread.join();
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

    /// Now we need to take care interacting with the target

    /// The thread will wait(without consuming CPU resources) until
    /// either the commandQueue is not empty, meaning there's something
    /// to process, or target_running becomes false, indicating a
    /// stop condition.

    /// If we do not need the command queue, we do not need
    /// all this locking and guarding - just wait until
    /// target_running becames false;
    while (manager_running) {
      /// start by locking the mutex. No other methods using this mutex
      /// can be executed until it is released
      std::unique_lock<std::mutex> lock(target_mutex);
      /// The wait method releases the lock and re-acquires it when
      /// the condition is met.
      commandCV.wait(lock, [this]() {  //
        return !commandQueue.empty() || !manager_running;
      });
      /// now we continue, with the mutex locked to ensure we have
      /// exclusive access while processing the command queue
      while (!commandQueue.empty()) {
        auto command = commandQueue.front();
        commandQueue.pop();
        if (command.type == ButtonPress) {
          target.digitalWrite(command.buttonID, command.state);
        } else if (command.type == UpdateLEDs) {
          //          auto ledStates = target.getPinState();
        }
      }
    }
  }

  void stopTarget() {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.stopRunning();
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
