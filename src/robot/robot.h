//
// Created by peter on 22/11/24.
//

#ifndef ROBOT_H
#define ROBOT_H

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>
#include "behaviour/profile.h"
#include "common/core.h"
#include "robot-state.h"
#include "sensor-data.h"

/***
 * The Robot class models the physical robot. It is just the vehicle that moves around the screen.
 *
 * It will provide the dynamic behaviour for the simulation
 *
 * It should be regarded as like your car. You can tell it to move and turn and you
 * can examine its status. Almost nothing about the Robot is specific to its use
 * for any particular behaviour except, perhaps, the non-holonomic constraints
 * and the array of sensor data it collects.
 *
 * Sensor data is actually evaluated by a callback to the application and stored
 * in the Robot. This is directly analogous to the way the sensors actually work.
 * The Application holds the information about the physical world and the
 * relationship of the robot to that world.
 * Provision is made for sensor data to simply be a number in the range 0..1023
 *
 * The interpretation of that number is something the Behaviour should be responsible
 * for.
 */
class Robot {
 public:
  Robot() : m_running(false) {
    //
  }

  ~Robot() {
    Stop();
  }

  void Start() {
    if (!m_running) {
      m_ticks = 0;
      m_running = true;
    }
  }

  void Stop() {
    if (m_running) {
      m_running = false;
    }
  }

  void Resume() {
    m_running = true;  //
  }

  ////// Accessors
  sf::Vector2f getPose() const {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return sf::Vector2f(m_state.x, m_state.y);
  }

  RobotState getState() const {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return m_state;
  }

  void setState(RobotState state) {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_state = state;
  }

  void resetState() {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_state = RobotState();
    m_state.x = 96;
    m_state.y = 96;
    m_state.theta = 90.0f;
  }

  float getOrientation() const {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return m_state.theta;
  }

  float getDistance() const {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return m_state.distance;
  }

  void setPosition(float x, float y) {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_state.x = x;
    m_state.y = y;
  }

  void setOrientation(float theta) {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_state.theta = theta;  //
  }

  void setVelocity(float velocity) {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_state.v = velocity;  //
  }

  void setOmega(float omega) {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_state.omega = omega;  //
  }

  void setAccel(float accel) {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_state.accel = accel;  //
  }

  void setAlpha(float alpha) {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_state.alpha = alpha;  //
  }

  ///////////// Sensors
  void setSensorCallback(SensorDataCallback callback) {
    m_sensor_callback = callback;  //
  }

  SensorData& getSensorData() {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return m_sensor_data;
  }

  uint32_t getTicks() const {
    return m_ticks;  //
  }
  void setTickCount(uint32_t mTicks) {
    m_ticks = mTicks;  //
  }

  const std::atomic<bool>& getRunning() const {
    return m_running;  //
  }

  void setSensorData(const SensorData& mSensorData) {
    m_sensor_data = mSensorData;  //
  }

  float getVMax() const {
    return m_vMax;  //
  }

  void setVMax(float mVMax) {
    m_vMax = fabsf(mVMax);  //
  }

  float getOmegaMax() const {
    return m_omegaMax;  //
  }

  void setOmegaMax(float mOmegaMax) {
    m_omegaMax = fabsf(mOmegaMax);  //
  }

  uint32_t millis() {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return m_ticks;
  }

  void setSpeeds(float velocity, float omega) {
    CRITICAL_SECTION(m_robot_mutex) {
      m_state.v = velocity;
      m_state.omega = omega;
    }
  }

  /***
   * The systick() method simulates an interrupt triggered by a timer on the hardware.
   *
   * This ISR normally handles all the IO, sensors, control systems and motion
   * processing on the physical robot.  The Systick normally runs at 1kHz on many
   * robots and so is also responsible for monitoring the passage of time, keeping
   * the main code synchronised with the hardware.
   * With the Behaviour separated out from the robot we can simply call the systick
   * ISR from the behaviour whenever we want the Robot to advance by one tick.
   *
   * Behaviour runs in a separate thread to the main application so, by extension,
   * does the Robot code. That means that Behaviour is free to interact with the
   * Robot without restraint, any calls from the Application to either Robot or
   * Behaviour must be guarded by a mutex or atomic variables.
   */

  void systick(float deltaTime) {
    // The mutex will lock out the main thread while this block runs.
    CRITICAL_SECTION(m_robot_mutex) {
      m_ticks++;
      // Ask the Application for a sensor update
      if (m_sensor_callback) {
        m_sensor_data = m_sensor_callback(m_state.x, m_state.y, m_state.theta);
      }

      // run the profilers

      // update the speeds
      m_state.v = std::clamp(m_state.v, -m_vMax, m_vMax);
      m_state.omega = std::clamp(m_state.omega, -m_omegaMax, +m_omegaMax);

      // accumulate distances
      float deltaDistance = m_state.v * deltaTime;
      m_profileDistance += deltaDistance;
      m_state.distance += deltaDistance;
      m_state.offset += deltaDistance;

      // wrap the angle
      m_state.theta += m_state.omega * deltaTime;
      if (m_state.theta >= 360.0f) {
        m_state.theta -= 360.0f;
      } else if (m_state.theta < 0.0f) {
        m_state.theta += 360.0f;
      }

      // calculate new location
      m_state.x += m_state.v * std::cos(m_state.theta * RADIANS) * deltaTime;
      m_state.y += m_state.v * std::sin(m_state.theta * RADIANS) * deltaTime;
    }
  }

 private:
  uint32_t m_ticks;
  std::atomic<bool> m_running;
  SensorDataCallback m_sensor_callback = nullptr;

  SensorData m_sensor_data;  // Current sensor readings
  RobotState m_state;
  // profile values
  float m_targetDistance = 0.0f;
  float m_maxVelocity = 0.0f;
  float m_targetVelocity = 0.0;
  float m_acceleration = 0.0f;
  float m_profileDistance = 0.0f;
  bool m_moving = false;

  float accelPhaseDistance = 0.0f;
  float decelPhaseDistance = 0.0f;
  float cruiseDistance = 0.0f;

  float m_vMax = 5000.0f;
  float m_omegaMax = 4000.0f;
};

#endif  // ROBOT_H
