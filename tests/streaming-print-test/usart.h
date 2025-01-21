//
// Created by peter on 21/01/25.
//

#pragma once

#include "common/core.h"
#include "print.h"

class USART : public Print {
 public:
  USART(SerialOut output_port)
      : m_serialOut(output_port) {
  }

  virtual size_t write(uint8_t c) {
    m_serialOut(c);
    return 1;
  };

 private:
  SerialOut m_serialOut;
};
