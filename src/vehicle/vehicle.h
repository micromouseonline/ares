// Vehicle.h
// Created by Peter on 22/11/24.
// Defines the Vehicle class which models the physical behavior of the robot.

#pragma once
#include <functional>
#include "common/core.h"
#include "common/pose.h"
#include "common/singleton.h"

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
};

using SensorDataCallback = std::function<VehicleInputs(VehicleState)>;

class Vehicle {
 public:
  Vehicle();
  ~Vehicle();

  /// used by MR32
  void begin();
  void setLed(const int i, const bool state);
  void setLedPattern(uint8_t pattern);
  void set_systick_callback(SystickMouseCallback callback);
  void set_steering_feedback(float steering_fb);
  float get_steering_feedback();
  void systick();
  void odometryUpdate();
  void reset_imu(int ms);
  void reset_drive_system();
  void enable_motors() {};
  void stop();
  void imu_calibrate(int samples = 500);  // not implemented
  float distance() const;
  float velocity() const;
  float angle() const;
  float omega() const;
  void setTargetVelocities(float velocity, float omega);
  void set_motor_voltage(float left, float right);
  float battery_voltage();
  bool has_panic();
  void panic(const char* message);
  bool hasButtonPressed();

  /// used by ARES
  void reset();
  void updateLeds();
  MotorVoltages motorControllersUpdate(Velocities desired, Velocities actual, float steering_feedback);
  VehicleState getState() const;
  void setPose(float x, float y, float angle);
  Pose getPose();
  void setSensorCallback(SensorDataCallback callback);
  //  void setSpeeds(float velocity, float omega);
  bool readButton(Button btn);
  uint8_t getButtons();
  void updateSensors();
  void updateMotion(float deltaTime);

 private:
  /// MR32
  Velocities desired_velocities;
  bool m_initialised = false;
  SystickMouseCallback systick_mouse_callback = nullptr;
  float m_steering_fb = 0.0f;
  bool m_has_panic = false;

  /// ARES
  Vehicle(const Vehicle&) = delete;
  Vehicle& operator=(const Vehicle&) = delete;
  SensorDataCallback m_sensor_callback = nullptr;
  VehicleState m_state;
  VehicleInputs m_inputs;
  float m_step_time = 0.001f;
};
