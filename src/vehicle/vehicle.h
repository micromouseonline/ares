// Vehicle.h
// Created by Peter on 22/11/24.
// Defines the Vehicle class which models the physical behavior of the robot.

#pragma once
#include <functional>
#include "common/core.h"
#include "common/pose.h"
#include "common/singleton.h"
#include "hal/
enum Button {
  BTN_GO = (1 << 0),
  BTN_RESET = (1 << 1),
};

enum Led {
  LED_1 = (1 << 0),
  LED_2 = (1 << 1),
  LED_3 = (1 << 2),
  LED_4 = (1 << 3),
};

struct SensorData {
  float lfs_distance = 0;
  float lds_distance = 0;
  float rds_distance = 0;
  float rfs_distance = 0;
  float lfs_power = 0;
  float lds_power = 0;
  float rds_power = 0;
  float rfs_power = 0;
  float front_sum = 0;
  float front_diff = 0;
  bool see_front_wall = false;
  bool see_left_wall = false;
  bool see_right_wall = false;
};

struct VehicleInputs {
  SensorData sensors;
  uint16_t adc[16] = {0};
  uint8_t buttons = 0;
};

struct VehicleState {
  uint32_t ticks = 0;
  float x = 0;
  float y = 0;
  float angle = 0;
  float velocity = 0;
  float angular_velocity = 0;
  float total_distance = 0;
  uint8_t leds = 0;
  uint8_t buttons = 0;
  SensorData sensors;
  uint16_t adc[16];
};

using SensorDataCallback = std::function<VehicleInputs(VehicleState)>;

class Vehicle {
 private:
  Vehicle();
  ~Vehicle();

 public:
  static Vehicle& instance() {
    static Vehicle instance;
    return instance;
  }
  /// used by MR32
  void init();

  /// systick will call a method in the Mouse class to update motion
  void setMouseCallback(SystickMouseCallback callback);
  /// systick updates all the vehicle hardware.
  bool systick();

  void resetDriveSystem();    /// disable controller output, clear all counters, stop motors
  void enableMotorOutput();   /// permits motor control voltage to be applied to motors
  void disableMotorOutput();  /// prevents motor control voltage from being applied to motors
  void stopMoving();          /// reset controller errors and stop the PWM

  /// The values measured by the Vehicle rather than the desired state
  float getDistance() const;
  float getVelocity() const;
  float getAngle() const;
  float getOmega() const;

  void resetIMU(int samples);  /// not needed
  /// reads encoders and IMU to update odometry
  void updateOdometry();

  void setTargetVelocities(float velocity, float omega);
  void setSteeringFeedback(float steering_fb);
  MotorVoltages updateMotorControlllers(Velocities desired, Velocities actual, float steering_feedback);
  void setMotorVoltage(float left, float right);  /// not needed in ARES

  float getBatteryVoltage();  /// interprets one of the ADC channels as battery voltage

  /// enter endless loop flashing LEDs an show a message
  void panic(const char* message);
  /// hardware will be in endless loop dusing panic. This is for compatibility with sim
  bool panicIsActive();

  void setLed(int i, bool state);       /// for setting an individual LED
  void setLedPattern(uint8_t pattern);  /// for setting first 8 LEDS on or off

  bool hasAnyButtonPressed();
  bool isButtonPressed(int button);

  /** Not available to ARES (yet)
     /// Hardware Abstraction Layer classes
     /// (or they will be )
     Speaker* speaker();
     Board* board();
     Display* display();
     Usart* serial();
     Button* button_x();
     Button* button_y();
     AnalogConverter* adc();
     Battery* battery();
     Gyro* gyro();
     Odometry* odometry();
     Motors* motors();
     MotorPWM* pwm();
   */

  /// used only by ARES //////////////////////////////////////////////////
  void reset();
  void pause();
  void resume();
  void terminate();
  bool isRunning();
  bool isPaused();
  VehicleState getState() const;
  void setPose(float x, float y, float angle);
  void setSensorCallback(SensorDataCallback callback);
  void updateSensors();                     /// Interprets sensor data from ADC - should be in Mouse
  void updateVehiclePose(float deltaTime);  /// updat profilers - should be in Mouse

 private:
  /// MR32
  Vehicle(const Vehicle&) = delete;
  Vehicle& operator=(const Vehicle&) = delete;

  Velocities desired_velocities;
  SystickMouseCallback m_SystickMouseCallback = nullptr;
  float m_steering_fb = 0.0f;
  bool m_has_panic = false;
  bool m_initialised = false;

  /// ARES

  bool m_terminate = false;
  bool m_reset = false;
  bool m_paused = false;

  SensorDataCallback m_SensorReadCallback = nullptr;
  VehicleState m_state;
  VehicleInputs m_inputs;
  float m_step_time = 0.001f;
};
