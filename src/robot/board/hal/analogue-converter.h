//
// Created by peter on 02/02/25.
//

#pragma once

/***
 * Stub for a 16 channel ADC converter used to get sensor readings
 * and the battery voltage.
 *
 * Conversions are stored as 16 bit unsigned integers in an array. The Vehicle
 * and Mouse have channel numbers to get to the information.
 *
 * NOTE: when updating MR32, it migh be a good idea to store the
 *       dark values in the first 8 elements and the lit values
 *       in the second 8 rather than keep lit and dark readings.
 *
 */
#include <stdint.h>

class AnalogueConverter {
 public:
  AnalogueConverter() {
  }

  void begin() {
    /// configure the ADC hardware for this device
  }

  void createChannel(int channelID) {
    /// this will configure a physical analogue pin and assign it to
    /// a position in the channel array
    (void)channelID;
  }

  uint16_t convertChannel(int channelID) {
    /// uses the ADC hardware to perform a conversion
    /// the channelID is the position in the array, not the
    /// hardware channel.
    /// Confusing terminology
    (void)channelID;
    return 0;
  }

  uint16_t getChannel(int channel) {
    return m_adc[channel];
  }

  uint16_t dark(int channel) {
    return m_adc[channel];
  }

  uint16_t lit(int channel) {
    return m_adc[channel];
  }

  uint16_t raw(int channel) {
    return lit(channel) - dark(channel);
  }

  /// TODO: these do not really belong here
  void enableEmitters() {
  }

  void disableEmitters() {
  }

  void update() {
    /// called from systick to do the conversions;
  }

 private:
  uint16_t m_adc[16] = {0};
};
