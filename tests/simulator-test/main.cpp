#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <thread>

// Command structure
enum CommandType { ButtonPress, UpdateLEDs };
struct Command {
  CommandType type;
  int buttonID;
  std::map<int, bool> ledStates;
};

// Constants
const int INPUT = 0;
const int OUTPUT = 1;
const bool HIGH = true;
const bool LOW = false;

// Plain function prototype for sensor readings
int sensorCallback(int pin);
/////////////////////////////////////////////////////////////////////////

// Simulated Arduino Nano Target class
class Target {
 public:
  volatile bool led10;
  volatile bool led11;
  int (*sensorCallback)(int);

  Target() : led10(false), led11(false), sensorCallback(nullptr) {
  }

  // Timer setup for 500Hz simulated using a member function
  void timerISR() {
    if (sensorCallback) {
      led10 = sensorCallback(10);
      led11 = sensorCallback(11);
    }
  }

  void setup() {
    pinMode(10, INPUT);
    pinMode(11, INPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);

    // Timer setup - use a different mechanism to simulate the timer interrupts in the simulation environment
    std::thread timerThread([this]() {
      while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));  // Simulate 500Hz timer
        timerISR();
      }
    });
    timerThread.detach();
  }

  void loop() {
    while (true) {
      digitalWrite(4, led10 ? HIGH : LOW);
      digitalWrite(5, led11 ? HIGH : LOW);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Add some delay for stability
    }
  }

  void setSensorCallback(int (*callback)(int)) {
    sensorCallback = callback;
  }

  std::map<int, bool> getLEDStates() {
    return {{4, led10}, {5, led11}};
  }

  void simulateButtonPress(int buttonID) {
    if (buttonID == 10) {
      led10 = !led10;
    } else if (buttonID == 11) {
      led11 = !led11;
    }
  }

  int digitalRead(int pin) {
    return (pin == 10) ? led10 : led11;
  }

  void digitalWrite(int pin, bool state) {
    std::cout << "Pin " << pin << " set to " << (state ? "HIGH" : "LOW") << std::endl;
  }

  void pinMode(int pin, int mode) {
    std::cout << "Pin " << pin << " set to mode " << mode << std::endl;
  }

  void noInterrupts() {
  }
  void interrupts() {
  }
};

/////////////////////////////////////////////////////////////////////////
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
  }

  ~Manager() {
    running = false;
    commandCV.notify_all();
    if (targetThread.joinable()) {
      targetThread.join();
    }
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

  void setSensorCallback(int (*callback)(int)) {
    target.setSensorCallback(callback);
  }
};
/////////////////////////////////////////////////////////////////////////

// Plain function for sensor readings
int sensorCallback(int pin) {
  return (pin == 10) ? HIGH : LOW;
}

// UI functions
void onButtonPress(int buttonID, Manager &manager) {
  manager.simulateButtonPress(buttonID);
}

void refreshUI(Manager &manager) {
  manager.updateLEDStates();
}

// Main function
int main() {
  Manager manager;

  // Set the sensor callback using a plain function
  manager.setSensorCallback(sensorCallback);

  std::thread uiThread([&manager]() {
    while (true) {
      onButtonPress(10, manager);
      refreshUI(manager);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });

  // Run for a limited time for demonstration
  std::this_thread::sleep_for(std::chrono::seconds(10));

  uiThread.detach();
  return 0;
}
