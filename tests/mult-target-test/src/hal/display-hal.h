/// Interface for a generic display HAL
///
/// This interface is used to abstract away the underlying display hardware.

/// display-hal.h (abstract sensor interface)
#pragma once

class DisplayHAL {
 public:
  virtual void initialize() = 0;
  virtual void cls() = 0;
  virtual void setCursor(int x, int y) = 0;
  virtual void write(const char c) = 0;
  virtual ~DisplayHAL() = default;
};
