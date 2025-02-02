//
// Created by peter on 02/02/25.
//

#pragma once

/***
 * stub for the Battery monitor
 */

class Battery {
 public:
  enum {
    DEAD,
    LOW,
    POOR,
    GOOD,
  };

  void begin(int ADC_channel) {
    m_adc_channel = ADC_channel;
  }

  void update() {
  }

  int check() {
    return GOOD;
  }

  int getVoltage() {
    /// normally this would be calculated form the ADC reading
    return 7625;  // mVolts
  }

 private:
  int m_adc_channel;
};
