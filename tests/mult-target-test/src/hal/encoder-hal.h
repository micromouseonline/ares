//
// Created by peter on 05/02/25.
//

#pragma once

#include <stdint.h>

class EncoderHAL {
 public:
  virtual void initialize() = 0;
  virtual void clear() = 0;
  virtual void update() = 0;
  virtual int32_t getCount() = 0;
  virtual int32_t getChange() = 0;
  virtual ~EncoderHAL() = default;
};
