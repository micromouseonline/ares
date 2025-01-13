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
  std::thread targetThread;
  std::atomic<bool> target_running;
  std::mutex target_mutex;
  std::queue<Command> commandQueue;
  std::condition_variable commandCV;

 public:
  Manager() : target_running(true) {
    targetThread = std::thread(&Manager::RunTarget, this);
    printf("Manager created\n");
  }

  ~Manager() {
    target_running = false;
    commandCV.notify_all();
    if (targetThread.joinable()) {
      targetThread.join();
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

    /// start the thread that the target runs in
    std::thread loopThread([this]() { target.mainLoop(); });

    /// Now we need to take care interacting with the target

    /// The thread will wait(without consuming CPU resources) until
    /// either the commandQueue is not empty, meaning there's something
    /// to process, or target_running becomes false, indicating a
    /// stop condition.
    while (target_running) {
      /// start by locking the mutex. No other methods using this mutex
      /// can be executed until it is released
      std::unique_lock<std::mutex> lock(target_mutex);
      /// The wait method releases the lock and re-acquires it when
      /// the condition is met.
      commandCV.wait(lock, [this]() {  //
        return !commandQueue.empty() || !target_running;
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

    loopThread.detach();
  }

  void setPinState(int i, bool state) {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.digitalWrite(i, state);
    target.digitalWrite(i, state);
  }
  //
  //  void setPinState(int i, bool state) {
  //    std::lock_guard<std::mutex> lock(target_mutex);
  //    target.digitalWrite(i, state);
  //    commandQueue.push({ButtonPress, i, state, {}});
  //    commandCV.notify_all();
  //  }

  std::array<bool, 16> getPins() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.getPinState();
  }

  SensorData getSensors() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.getSensors();
  }

  void updateLEDStates() {
    std::lock_guard<std::mutex> lock(target_mutex);
    commandQueue.push({UpdateLEDs, 0, false, {}});
    commandCV.notify_all();
  }

  void setSensorCallback(SensorCallbackFunction cb) {
    target.setSensorCallback(cb);
  }
};
