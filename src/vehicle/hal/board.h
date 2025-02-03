//
// Created by peter on 02/02/25.
//

#pragma once

/***
 * the board is a singleton that looks after the hardware
 *
 */
#include <stdio.h>
#include "analogue-converter.h"
#include "battery.h"
#include "board-config.h"
#include "button.h"
#include "display.h"
#include "gyro.h"
#include "motor-controller.h"
#include "odometry.h"
#include "pwm.h"
#include "speaker.h"
#include "usart.h"

/// TODO: this need to be an interface class for actual boards
class Board {
 public:
  Board() {
    init();
    printf("Board created\n");
  }

  ~Board() {
    printf("Board destroyed\n");
  }

  /// initialise all the hardware
  void init() {
    /// set the system clock
    /// begin the components
    /// create the ADCchannels
    /// initialise the gyro
    /// kick off systick
    printf("Board initialiseing\n");
  }

  /***
   * Called from systick, this will update the state of the peripherals
   * along with any local shadow variables like the LED states
   */
  void update() {
  }

  /// Buttons
  bool hasAnyButtonPressed() {
    for (int i = 0; i < BUTTON_COUNT; i++) {
      if (m_buttons[i].isPressed()) {
        return true;
      }
    }
    return false;
  };

  bool isButtonPressed(ButtonID button) {
    return m_buttons[button].isPressed();
  };

  /// LEDs
  void setLed(LedID led, bool state) {
    m_leds[led] = state;
  }

  /// Serial device
  void serialWrite(const char c) {
    m_serial.write(c);
  }
  int serialPuts(const char* s) {
    return m_serial.puts(s);
  }

  /// Display Device
  void displayWrite(const char c) {
    m_display.write(c);
  }

  int displayPuts(const char* s) {
    return m_display.puts(s);
  }

  void displayCLS() {
    return m_display.cls();
  }

  /// ADC data
  uint16_t getAdcChannel(int channel) {
    return m_adc_data[channel];
  }

  /// Battery
  float getBatteryVoltage() {
    return m_battery.getVoltage();
  }

  /// Motors
  void setMotorVoltages(float left, float right) {
  }

  MotorVoltages getMotorVoltages() {
    return {0, 0};
  }

  /// Odometry

  void resetOdometry() {
  }

  int16_t getLeftCount() {
    return 0;
  }

  int16_t getRightCount() {
    return 0;
  }

  /// Gyro
  float getImuYawRate() {
    return 0.0;
  }

  /// Speaker
  void playTone(uint32_t frequency, uint32_t duration) {
    m_speaker.playTone(frequency, duration);
  }

  void beep(uint16_t duration) {
    m_speaker.playTone(1000, 100);
  }

 private:
  Speaker m_speaker;
  AnalogueConverter m_adc;
  Odometry m_odometry;
  Battery m_battery;
  Gyro m_gyro;
  Display m_display;
  Usart m_serial;  //  this is a pointer to an existing usart device
  Button m_buttons[BUTTON_COUNT];
  bool m_leds[LED_COUNT];
  uint16_t m_adc_data[ADC_CHANNEL_COUNT];

  /// make sure we ca create no copies
  Board& operator=(const Board) = delete;
  Board(const Board&) = delete;
  Board& operator=(const Board&) = delete;
  Board(Board&&) = delete;
  Board& operator=(Board&&) = delete;
};
