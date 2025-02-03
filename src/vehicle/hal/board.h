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
#include "button.h"
#include "display.h"
#include "gyro.h"
#include "motor-controller.h"
#include "odometry.h"
#include "pwm.h"
#include "speaker.h"
#include "usart.h"

class Board {
 public:
  static Board* instance() {
    static Board instance;
    return &instance;
  }

  Usart& serial() {
    return m_serial;
  }

  /// TODO: combine the buttons?
  ButtonQ* button_x() {
    return &m_button_x;
  }
  ButtonQ* button_y() {
    return &m_button_y;
  }

  AnalogueConverter* adc() {
    return &m_adc;
  }

  Battery* battery() {
    return &m_battery;
  }

  Gyro* gyro() {
    return &m_gyro;
  }

  Display* display() {
    return &m_display;
  }

 private:
  ButtonQ m_button_x;
  ButtonQ m_button_y;
  Speaker m_speaker;
  AnalogueConverter m_adc;
  Battery m_battery;
  Gyro m_gyro;
  Display m_display;
  Usart m_serial;  //  this is a pointer to an existing usart device

  Board() {
    /// set the system clock
    /// begin the components
    /// create the ADCchannels
    /// initialise the gyro
    /// kick off systick
    printf("Board created\n");
  }

  /// make sure we ca create no copies
  Board& operator=(const Board) = delete;
  Board(const Board&) = delete;
  Board& operator=(const Board&) = delete;
  Board(Board&&) = delete;
  Board& operator=(Board&&) = delete;
};
