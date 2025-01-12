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
  std::atomic<bool> running;
  std::mutex commandMutex;
  std::queue<Command> commandQueue;
  std::condition_variable commandCV;

 public:
  Manager() : running(true) {
    targetThread = std::thread(&Manager::RunTarget, this);
    printf("Manager created\n");
  }

  ~Manager() {
    running = false;
    commandCV.notify_all();
    if (targetThread.joinable()) {
      targetThread.join();
    }
    printf("Manager destroyed\n");
  }

  void RunTarget() {
    target.setup();
    std::thread loopThread([this]() { target.loop(); });
    while (running) {
      std::unique_lock<std::mutex> lock(commandMutex);
      commandCV.wait(lock, [this]() { return !commandQueue.empty() || !running; });
      while (!commandQueue.empty()) {
        auto command = commandQueue.front();
        commandQueue.pop();
        if (command.type == ButtonPress) {
          target.simulateButtonPress(command.buttonID);
        } else if (command.type == UpdateLEDs) {
          // Handle LED state updates
          auto ledStates = target.getLEDStates();
        }
      }
    }
    loopThread.detach();
  }

  void simulateButtonPress(int buttonID) {
    std::lock_guard<std::mutex> lock(commandMutex);
    commandQueue.push({ButtonPress, buttonID, {}});
    commandCV.notify_all();
  }

  void updateLEDStates() {
    std::lock_guard<std::mutex> lock(commandMutex);
    commandQueue.push({UpdateLEDs, 0, {}});
    commandCV.notify_all();
  }

  void setSensorCallback(SensorCallbackFunction cb) {
    target.setSensorCallback(cb);
  }
};
