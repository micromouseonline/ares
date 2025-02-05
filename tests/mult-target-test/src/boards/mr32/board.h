

#pragma once

// Board-specific includes (e.g., SPI, GPIO, UART drivers)
#include "display/matrix/HCM3906.h"
#include "imu/mr32/imu-mr32.h"

// Define hardware-specific parameters for Ares
#define BOARD_NAME "MR32"
#define LED_PIN 13
#define SPI_SPEED 1000000  // SPI baud rate for Ares board
#define USE_UART 1

// Ares-specific function definitions or configuration settings
void initPeripherals() {
  std::cout << "initialising board for " << BOARD_NAME << std::endl;
  
  // Initialize Ares peripherals like SPI, UART, GPIOs, etc.
  // For example: Initialize SPI for Ares
  //  SPI_Init(SPI_SPEED);
}
