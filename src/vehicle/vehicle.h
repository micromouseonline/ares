// robot.h
// Created by peter on 22/11/24.
// Defines the Robot class which models the physical behaviour of the robot
//
#pragma once
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
// #include <mutex>
#include "behaviour/trajectories/straight.h"
#include "common/core.h"
#include "vehicle-state.h"

/**
 * @brief The Vehicle class models the physical robot's behavior and movement.
 *
 * The Vehicle class acts as a simulated vehicle that moves and turns on the screen,
 * mimicking the dynamic behavior of a real-world robot. It is designed to be
 * generic and independent of any specific behavior or control logic, except for
 * non-holonomic constraints and sensor data collection.
 *
 * ### Key Features:
 * - **Movement and State:**
 *   The robot's state (position, orientation, and velocity) is maintained in a
 *   single struct. The forward and angular velocities are the primary inputs,
 *   and the robot assumes ideal instantaneous matching of these values.
 *
 * - **Sensor Data:**
 *   Sensor readings are evaluated via a callback function provided by the
 *   application. The Robot stores the sensor data as raw values (0-1023), while
 *   interpreting these values is left to the Behavior layer. This design mimics
 *   real-world sensor operation where the robot queries the environment but
 *   delegates interpretation to higher-level logic.
 *
 * - **Virtual inputs and monitoring:**
 *   When the Vehicle requests sensor data it sends a copy of its current state
 *   to the application. That way the application can monitor the vehicle without
 *   having to call any of its methods. For example, the state of the indicator
 *   LEDS is read to updatethe application UI.
 *
 *   The application sends back the sensor data as part of another complete copy
 *   of the vehicle state. The main reason is to allow the application to perform
 *   tasks like simulate the pressing of buttons on the vehicle or update the
 *   settings switches or their equivalent
 *
 * - **Simulation of Hardware Interrupts:**
 *   The `updateMotion()` method simulates a hardware timer interrupt, typically running
 *   at 1kHz on real robots. It updates low-level controllers, monitors encoders
 *   and sensors, and handles motion processing. This ensures accurate time
 *   synchronization and hardware-like behavior.
 *
 * - **Time Management:**
 *   The Robot maintains a millisecond-accurate counter (`m_state.ticks`), which serves
 *   as a timestamp for logging and timing purposes.
 *
 * ### Relationship with Application and Behavior:
 * - The **Application** holds the physical world data and provides the sensor
 *   callback, determining the robot's relationship with the environment.
 * - The **Behavior** layer interacts with the Robot to control its movement,
 *   interpret sensor data, and define higher-level actions.
 *
 * ### Multi-threaded running
 *   The behaviour and the vehicle code run in a shared thread
 *   Thus the behaviour code (the muse) is able to call any vehicle method
 *   without having to worry about shared data and mutexes
 *   The Robot manager and the application both run in a different thread
 *   and so should not call any vehicle methods while the vehicle is running
 *
 * This separation of concerns ensures modularity, where the Vehicle focuses solely
 * on physical behavior, and the Behavior or Application layers handle interpretation
 * and control logic.
 */

class Vehicle {
 public:
  Vehicle() : m_state() {
    reset();
  }

  ~Vehicle() {
  }

  void reset() {
    m_state.ticks = 0;
    m_state.total_distance = 0;
    setSpeeds(0, 0);
  }

  /// This is safe to call only from the behaviour (mouse) code
  [[nodiscard]] VehicleState getState() const {
    return m_state;
  }

  /// This is safe to call only from the behaviour (mouse) code
  void setPose(float x, float y, float angle) {
    m_state.x = x;
    m_state.y = y;
    m_state.angle = angle;
  }

  /// This is safe to call only from the behaviour (mouse) code
  Pose getPose() {
    Pose pose;
    pose.setX(m_state.x);
    pose.setY(m_state.y);
    pose.setAngle(m_state.angle);
    return pose;
  }

  /// gets called from the application before the robot thread starts
  void setSensorCallback(SensorDataCallback callback) {
    m_sensor_callback = callback;  //
  }

  /// Once set, speeds will not change unless commanded
  /// This is safe to call only from the behaviour (mouse) code
  void setSpeeds(float velocity, float omega) {
    m_state.velocity = velocity;
    m_state.angular_velocity = omega;
  }

  /// This is safe to call only from the behaviour (mouse) code
  void setLed(const int i, const bool state) {
    const uint8_t mask = BIT(i);
    m_state.leds &= ~(mask);
    m_state.leds |= state ? mask : 0;
  }

  /// This is safe to call only from the behaviour (mouse) code
  bool readButton(int btn) {
    return ((m_inputs.buttons & btn) != 0);
  }

  uint8_t getButtons() {
    return m_inputs.buttons;
  }

  void clearButtons() {
    m_inputs.buttons = 0;
    m_state.buttons = 0;
  }

  /// this should be a single method that updates the state of the vehicle
  /// inputs like buttons, sensors, IMU, activity setting...
  /// the LEDS are included because of the simulation. They are better set
  /// directly on the target hardware.
  /// Conditional compilation can distinguish the source of this data
  void updateSensors() {
    if (m_sensor_callback) {
      m_inputs = m_sensor_callback(m_state);
      m_state.sensors = m_inputs.sensors;
    }
  }

  /// always call after sensors
  void updateInputs() {
    m_state.buttons = m_inputs.buttons;
    m_state.leds = m_inputs.leds;
    m_state.activity = m_inputs.activity;
    m_activity = m_inputs.activity;
    m_activity_arg = m_inputs.activity_arg;
  }
  /***
   * In this simulation, the updateMotion() method is invoked from the Behaviour class
   * to advance the Robot's state by one tick. Since Behaviour runs in a separate
   * thread from the main application, the Vehicle code executes in its same thread.
   * This design allows Behaviour to interact with the Robot freely, but any calls
   * from the Application to Robot or Behaviour must be thread-safe, using mutexes
   * or atomic variables.
   */
  void updateMotion(float deltaTime) {
    m_state.ticks++;
    /// The speeds are set directly from the behaviour code and now are
    /// used to update the vehicle pose.
    float deltaDistance = m_state.velocity * deltaTime;
    float deltaAngle = m_state.angular_velocity * deltaTime;

    float newX = m_state.x + deltaDistance * std::cos(m_state.angle * RADIANS);
    float newY = m_state.y + deltaDistance * std::sin(m_state.angle * RADIANS);
    float newAngle = std::fmod(m_state.angle + deltaAngle + 360.0f, 360.0f);

    m_state.total_distance += deltaDistance;
    m_state.x = newX;
    m_state.y = newY;
    m_state.angle = newAngle;
  }

 private:
  Vehicle(const Vehicle&) = delete;             /// no copying
  Vehicle& operator=(const Vehicle&) = delete;  /// no copying by assignment
                                                //  bool m_running;
  SensorDataCallback m_sensor_callback = nullptr;
  VehicleState m_state;
  VehicleInputs m_inputs;
  int m_activity;
  int m_activity_arg;
};
