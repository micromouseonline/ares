#include "manager.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "target.h"

// Plain function prototype for sensor readings
int sensorCallback(int pin);
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////

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
