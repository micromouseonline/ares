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
  std::mutex commandMutex;
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

  void RunTarget() {
    target.setup();
    std::thread loopThread([this]() { target.mainLoop(); });
    while (target_running) {
      std::unique_lock<std::mutex> lock(commandMutex);
      commandCV.wait(lock, [this]() {  //
        return !commandQueue.empty() || !target_running;
      });
      while (!commandQueue.empty()) {
        auto command = commandQueue.front();
        commandQueue.pop();
        if (command.type == ButtonPress) {
          target.digitalWrite(command.buttonID, command.state);
        } else if (command.type == UpdateLEDs) {
          auto ledStates = target.getPinState();
        }
      }
    }
    loopThread.detach();
  }

  void setPinState(int buttonID, bool state) {
    std::lock_guard<std::mutex> lock(commandMutex);
    commandQueue.push({ButtonPress, buttonID, state, {}});
    commandCV.notify_all();
  }

  void updateLEDStates() {
    std::lock_guard<std::mutex> lock(commandMutex);
    commandQueue.push({UpdateLEDs, 0, false, {}});
    commandCV.notify_all();
  }

  void setSensorCallback(SensorCallbackFunction cb) {
    target.setSensorCallback(cb);
  }
};
