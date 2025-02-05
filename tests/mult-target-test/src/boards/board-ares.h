

#pragma once

// Board-specific includes (e.g., SPI, GPIO, UART drivers)
#include "drivers/display/oled-128x64.h"
#include "drivers/imu/imu-ares.h"

// Define hardware-specific parameters for Ares
#define BOARD_NAME "ARES"
#define LED_PIN 13
#define SPI_SPEED 1000000  // SPI baud rate for Ares board
#define USE_UART 1

DisplayOLED display;
IMUAres imu;

// Ares-specific function definitions or configuration settings
void initPeripherals() {
  std::cout << "initialising board for " << BOARD_NAME << std::endl;
  imu.initialize();
  display.initialize();
  // Initialize Ares peripherals like SPI, UART, GPIOs, etc.
  // For example: Initialize SPI for Ares
  //  SPI_Init(SPI_SPEED);
}
