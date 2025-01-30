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
  uint16_t adc[16];
};

using SensorDataCallback = std::function<VehicleInputs(VehicleState)>;

class Vehicle {
 public:
  Vehicle();
  ~Vehicle();

  /// used by MR32
  void init();

  void setSystickCallback(SystickMouseCallback callback);
  void systick();

  void stopMoving() {};
  void enableMotorOutput() {};   /// Not needed
  void disableMotorOutput() {};  /// Not needed
  void resetDriveSystem();       /// implement this

  /// These are the values measured by the Vehicle rather than the desired state
  float getDistance() const;
  float getVelocity() const;
  float getAngle() const;
  float getOmega() const;

  void imuReset(int samples);  /// not needed
  void odometryUpdate();       /// reads encoders and IMU to update odometry

  void setTargetVelocities(float velocity, float omega);
  void setSteeringFeedback(float steering_fb);
  MotorVoltages motorControllersUpdate(Velocities desired, Velocities actual, float steering_feedback);
  void setMotorVoltage(float left, float right);  /// not needed

  float battery_voltage();
  bool has_panic();
  void panic(const char* message);

  void setLed(const int i, const bool state);
  void setLedPattern(uint8_t pattern);

  bool hasButtonPressed();
  bool isButtonPressed(int button);

  /// used by ARES //////////////////////////////////////////////////
  void reset();
  void updateLeds();
  VehicleState getState() const;
  void setPose(float x, float y, float angle);
  Pose getPose();
  void setSensorCallback(SensorDataCallback callback);
  bool readButton(Button btn);
  uint8_t getButtons();
  void updateSensors();
  void updateMotion(float deltaTime);

 private:
  /// MR32
  Velocities desired_velocities;
  SystickMouseCallback systick_mouse_callback = nullptr;
  float m_steering_fb = 0.0f;
  bool m_has_panic = false;
  bool m_initialised = false;

  /// ARES
  Vehicle(const Vehicle&) = delete;
  Vehicle& operator=(const Vehicle&) = delete;
  SensorDataCallback m_sensor_callback = nullptr;
  VehicleState m_state;
  VehicleInputs m_inputs;
  float m_step_time = 0.001f;
};
