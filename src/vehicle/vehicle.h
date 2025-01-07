// robot.h
// Created by peter on 22/11/24.
// Defines the Robot class which models the physical behaviour of the robot
//
#pragma once
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>
#include "behaviour/trapezoid.h"
#include "common/core.h"
#include "common/pose.h"
#include "sensor-data.h"
#include "vehicle-state.h"

/**
 * @brief The Robot class models the physical robot's behavior and movement.
 *
 * The Robot class acts as a simulated vehicle that moves and turns on the screen,
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
 * - **Simulation of Hardware Interrupts:**
 *   The `systick()` method simulates a hardware timer interrupt, typically running
 *   at 1kHz on real robots. It updates low-level controllers, monitors encoders
 *   and sensors, and handles motion processing. This ensures accurate time
 *   synchronization and hardware-like behavior.
 *
 * - **Time Management:**
 *   The Robot maintains a millisecond-accurate counter (`m_ticks`), which serves
 *   as a timestamp for logging and timing purposes.
 *
 * - **Data Logging:**
 *   The systick method can also perform black-box style data logging, recording
 *   low-level behavior for debugging or analysis.
 *
 * ### Relationship with Application and Behavior:
 * - The **Application** holds the physical world data and provides the sensor
 *   callback, determining the robot's relationship with the environment.
 * - The **Behavior** layer interacts with the Robot to control its movement,
 *   interpret sensor data, and define higher-level actions.
 *
 * This separation of concerns ensures modularity, where the Robot focuses solely
 * on physical behavior, and the Behavior or Application layers handle interpretation
 * and control logic.
 */
class Vehicle {
 public:
  const float VELOCITY_MAX = 8000.0f;
  const float OMEGA_MAX = 4000.0f;

  Vehicle() : m_ticks(0), m_running(false), m_state(), m_vMax(VELOCITY_MAX), m_omegaMax(OMEGA_MAX) {
    //
  }

  ~Vehicle() {
    stop();
  }

  void start() {
    if (!m_running) {
      m_ticks = 0;
      m_running = true;
    }
  }

  void stop() {
    if (m_running) {
      m_running = false;
    }
  }

  void resume() {
    m_running = true;  //
  }

  ////// Accessors

  [[nodiscard]] VehicleState getState() const {
    return m_state;
  }

  void setState(VehicleState state) {
    m_state = state;
  }

  void setPose(float x, float y, float angle) {
    m_state.x = x;
    m_state.y = y;
    m_state.angle = angle;

    m_pose.setX(x);
    m_pose.setY(y);
    m_pose.setTheta(angle);
  }

  ///////////// Sensors
  void setSensorCallback(SensorDataCallback callback) {
    m_sensor_callback = callback;  //
  }

  /// This is the primary way to get the robot to move.
  /// A more physics-based model would need accelerations.
  /// Here we assume the controllers are really good and that
  /// only realistic demands are made of the robot.
  /// Once set, speeds will not change unless comaanded
  void setSpeeds(float velocity, float omega) {
    m_state.velocity = velocity;
    m_state.omega = omega;
    // TODO: should this be clamped here or handled elsewhere?
    m_state.velocity = std::clamp(m_state.velocity, -m_vMax, m_vMax);
    m_state.omega = std::clamp(m_state.omega, -m_omegaMax, +m_omegaMax);
    m_pose.setVelocity(velocity);
    m_pose.setOmega(omega);
  }

  void adjustCellOffset(float delta) {
    m_state.cell_offset += delta;
  }

  void setCellOffset(float offset) {
    m_state.cell_offset = offset;
  }

  void resetMoveDistance() {
    m_state.move_distance = 0.0f;
  }

  void resetTotalDistance() {
    m_state.total_distance = 0.0f;
  }

  void resetMoveAngle() {
    m_state.move_angle = 0.0f;
  }

  [[nodiscard]] bool isRunning() const {
    return m_running;
  }

  [[nodiscard]] uint32_t millis() const {
    return m_ticks;
  }

  void setLed(const int i, const bool state) {
    const uint8_t mask = BIT(i);
    m_state.leds &= ~(mask);
    m_state.leds |= state ? mask : 0;
  }
  //
  //  [[nodiscard]] bool getLed(const int i) const {
  //    const uint8_t mask = BIT(i);
  //    return (m_leds & mask) == mask;
  //  }
  //
  //  [[nodiscard]] uint8_t getLeds() const {
  //    return m_leds;
  //  }

  void setButton(const int i, const bool state) {
    const uint8_t mask = BIT(i);
    m_state.buttons &= ~(mask);
    m_state.buttons |= state ? mask : 0;
  }
  //
  //  [[nodiscard]] bool isButton(const int i) const {
  //    const uint8_t mask = BIT(i);
  //    return (m_buttons & mask) != 0;
  //  }
  //
  //  [[nodiscard]] uint8_t getButtons() const {
  //    return m_buttons;
  //  }

  /**
   * @brief Simulates a hardware timer interrupt for the robot.
   *
   * The systick() method acts as a simulated ISR (Interrupt Service Routine) that
   * updates the robot's state, sensors, and motion processing. On physical robots,
   * such a timer ISR typically runs at 1kHz, handling IO operations, control systems,
   * and time synchronization with hardware.
   *
   * In this simulation, the systick() method is invoked from the Behaviour class
   * to advance the Robot's state by one tick. Since Behaviour runs in a separate
   * thread from the main application, the Robot code also executes in its own thread.
   * This design allows Behaviour to interact with the Robot freely, but any calls
   * from the Application to Robot or Behaviour must be thread-safe, using mutexes
   * or atomic variables.
   */

  void systick(float deltaTime) {
    if (!isRunning()) {
      return;
    }

    // Temporary variables for calculations outside the critical section
    VehicleState localState;
    SensorData sensorData;

    // Critical section: read the state and increment ticks
    {
      m_ticks++;
      m_state.timestamp = m_ticks;

      // Copy the current state for calculations
      localState = m_state;
    }

    // Perform calculations outside the critical section
    float deltaDistance = localState.velocity * deltaTime;
    float deltaAngle = localState.omega * deltaTime;

    float newX = localState.x + deltaDistance * std::cos(localState.angle * RADIANS);
    float newY = localState.y + deltaDistance * std::sin(localState.angle * RADIANS);
    float newAngle = std::fmod(localState.angle + deltaAngle + 360.0f, 360.0f);

    // If a sensor callback is set, get sensor data (this may involve external threads)
    if (m_sensor_callback) {
      sensorData = m_sensor_callback(localState.x, localState.y, localState.angle);
    }

    // Critical section: update the state with new calculations
    {
      m_state.total_distance += deltaDistance;
      m_state.cell_offset += deltaDistance;    // TODO: should be a behavior thing
      m_state.move_distance += deltaDistance;  // TODO: NOT USED

      m_state.x = newX;
      m_state.y = newY;
      m_state.angle = newAngle;

      if (m_sensor_callback) {
        m_state.sensor_data = sensorData;
      }

      // Advance the pose with the new delta time
      m_pose.advance(deltaTime);
    }
  }

 private:
  Vehicle(const Vehicle&) = delete;
  Vehicle& operator=(const Vehicle&) = delete;

  uint32_t m_ticks;
  std::atomic<bool> m_running;

  SensorDataCallback m_sensor_callback = nullptr;

  VehicleState m_state;
  Pose m_pose;
  float m_vMax;
  float m_omegaMax;
};
