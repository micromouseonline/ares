
#include <chrono>
#include <iostream>
#include <thread>

#include "application.h"

// Plain function prototype for sensor readings

// Main function
int main() {
  Application app;
  app.run();

  // Run for a limited time for demonstration
  // std::this_thread::sleep_for(std::chrono::seconds(3));

  return 0;
}
