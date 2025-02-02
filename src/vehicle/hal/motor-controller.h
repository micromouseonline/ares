//
// Created by peter on 02/02/25.
//

#pragma once

/***
 * Stub for the motor drive
 */

class MotorController {
 public:
  struct Velocities {
    float velocity;
    float omega;
  };

  struct Voltages {
    float left;
    float right;
  };

  void begin() {
  }

  void reset() {
  }

  float updatePositionControl(float desired, float actual) {
  }

  float updateRotationControl(float desired, float actual) {
  }

  float calculateforwardFeedForward(float velocity) {
  }

  float calculaterotationFeedForward(float omega) {
  }

  /// Called from systick to generate the motor voltages
  Voltages calculateMotorVoltages(Velocities desired, Velocities actual, float steering_adjust) {
  }

  void enableFeedForward() {
  }

  void disableFeedForward() {
  }

  float getLeftVolts() {
    return m_left_volts;
  }

  float getRightVolts() {
    return m_right_volts;
  }

  /// these are simple PD controllers a class would be better
  Voltages getControllerOutputs() {
  }

  Voltages getFeedForwardOutputs() {
  }

  float getForwardError() {
  }

  float getRotationError() {
  }

  /// TODO rename as connectOutput and disconnectOutput
  void enableControllerOutput() {
  }

  void disableControllerOutput() {
  }

  bool isConnected() {
  }

 private:
  bool m_controller_output_enabled = false;
  bool m_feedforward_enabled = true;

  float m_previous_fwd_error;
  float m_fwd_error;

  float m_previous_rot_error;
  float m_rot_error;

  float m_fwd_ff;
  float m_rot_ff;

  float m_left_volts;
  float m_right_volts;

  float m_fwd_control_volts;
  float m_rot_control_volts;
};
