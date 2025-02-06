

#pragma once

// Board-specific includes (e.g., SPI, GPIO, UART drivers)
#include "app/display.h"
#include "drivers/display/oled-128x64.h"
#include "drivers/imu/imu-ares.h"
#include "drivers/stm32/stm32-encoder.h"
#include "vehicle/encoder.h"
#include "vehicle/odometry.h"
// Define hardware-specific parameters for Ares
#define BOARD_NAME "ARES"
#define LED_PIN 13
#define SPI_SPEED 1000000  // SPI baud rate for Ares board
#define USE_UART 1

DisplayOLED oled;
Display display(oled);
IMUAres imu;
int32_t T2_COUNTER;
STM32Encoder left_encoder(T2_COUNTER);
Encoder left_input(left_encoder);

// Ares-specific function definitions or configuration settings
void initPeripherals() {
  std::cout << "initialising board for " << BOARD_NAME << std::endl;
  imu.initialize();
  display.initialize();
  left_encoder.initialize();
  // Initialize Ares peripherals like SPI, UART, GPIOs, etc.
  // For example: Initialize SPI for Ares
  //  SPI_Init(SPI_SPEED);
}
