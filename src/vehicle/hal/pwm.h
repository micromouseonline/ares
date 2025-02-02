//
// Created by peter on 02/02/25.
//

#pragma once

/***
 * A stub for the motor PWM driver.
 *
 * Normally this would get desired motor voltages and use the battery
 * voltage to calculate a PWM percentage for each motor.
 *
 * This is used internally by the Vehicle but exposed for debugging.
 *
 * It can also be used durectly for force feedback
 */

class MotorPWM {
 public:
  void begin() {
    /// configure the PWM hardware;
  }

  void setVoltage(float left, float right) {
    setLeftVolts(left);
    setRightVolts(right);
  }

  void setLeftVolts(float volts) {
    /// broken out for testing and force feedback
  }

  void setRightVolts(float volts) {
    /// broken out for testing and force feedback
  }

  void setPWM(int left, int right) {
    setLeftPWM(left);
    setRightPWM(right);
  }

  void setLeftPWM(int pwm) {
    /// talk to the hardware, setting actual PWM signal
  }

  void setRightPWM(int pwm) {
    /// talk to the hardware, setting actual PWM signal
  }
};
