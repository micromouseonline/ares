//
// Created by peter on 03/02/25.
//

#pragma once

enum ButtonID { BTN_GO = 0, BTN_RESET = 1, BUTTON_COUNT = 8 };

enum LedID {
  LED_1 = (1 << 0),
  LED_2 = (1 << 1),
  LED_3 = (1 << 2),
  LED_4 = (1 << 3),
};

const int ADC_CHANNEL_COUNT = 16;
const int LED_COUNT = 16;
