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
#include "board-interface.h"
#include "button.h"
#include "display.h"
#include "gyro.h"
#include "motor-controller.h"
#include "odometry.h"
#include "pwm.h"
#include "speaker.h"
#include "usart.h"

/// TODO: this need to be an interface class for actual boards
class AresBoard : public BoardInterface {
 public:
  static AresBoard& getInstance(void* params = nullptr) {
    static AresBoard instance(params);  // Meyers Singleton
    return instance;
  }

  AresBoard(void* params) {
    name = "ARES BOARD";
    init();
    //    printf("AresBoard created\n");
  }

  ~AresBoard() {
    //    printf("AresBoard destroyed\n");
  }

  /// initialise all the hardware
  void init() {
    /// set the system clock
    /// begin the components
    /// create the ADCchannels
    /// initialise the gyro
    /// kick off systick
    //    printf("AresBoard initialiseing\n");
  }

  /***
   * Called from systick, this will update the state of the peripherals
   * along with any local shadow variables like the LED states
   */
  void update() {
  }

  /// Buttons
  bool isAnyButtonPressed() override {
    for (int i = 0; i < BUTTON_COUNT; i++) {
      if (m_buttons[i].isPressed()) {
        return true;
      }
    }
    return false;
  };

  bool isButtonPressed(int button) override {
    return m_buttons[button].isPressed();
  };

  /// LEDs
  void setLed(int led, bool state) override {
    m_leds[led] = state;
  }

  /// Serial device
  void serialWrite(const char c) override {
    m_serial.write(c);
  }

  void serialPuts(const char* s) override {
    m_serial.puts(s);
  }

  /// Display Device
  void displayWrite(const char c) override {
    m_display.write(c);
  }

  void displayPuts(const char* s) {
    m_display.puts(s);
  }

  void displayCLS() {
    m_display.cls();
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
    (void)left;
    (void)right;
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
  void playTone(uint32_t frequency, uint32_t duration) override {
    m_speaker.playTone(frequency, duration);
  }

  void beep(int duration) override {
    m_speaker.playTone(1000, 100);
  }

 private:
  /// These should ideally have constructors that initialise them
  /// and they should only
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
  AresBoard& operator=(const AresBoard) = delete;
  AresBoard(const AresBoard&) = delete;
  AresBoard& operator=(const AresBoard&) = delete;
  AresBoard(AresBoard&&) = delete;
  AresBoard& operator=(AresBoard&&) = delete;
};
