// robot.h
// Created by peter on 22/11/24.
// Defines the Robot class which models the physical behaviour of the robot
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
 *The interpretation of that number is something the Behaviour should be responsible
 * for.
 * Distance is also held and made available.
 *
 * Robot state is held in a single struct that comprises current speed, orientation
 * and position.
 *
 * A real robot might also provide data from the control system such as servo errors
 * and motor voltages.
 *
 * The inputs to the robot are few. For simulation purposes, we only need to give the
 * forward and angular velocities. It is assumed that the (ideal) robot body can match
 * those immediately.
 *
 * To maintain some similarity with the physical robot, there is a systick method that
 * would normally be called by an interrupt at some fixed frequency. Systick would
 * normally update the low-level controllers, monitor the encoders and IMU, and
 * possibly look after buttons and LEDS.
 *
 * Black-box style data logging would also be performed from systick to get a running
 * record of the low level behaviour or the robot.
 *
 * For synchronisation purposes, the Robot maintains a millisecod-accurate counter. The
 * value of that counter is used as the timestamp for all logging and timing in the system.
 *
 *
 */
class Robot {
 public:
  Robot() : m_running(false), m_vMax(8000.0f), m_omegaMax(4000.0f) {
    //
    stop();
  }

  ~Robot() {
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
  sf::Vector2f getPose() const {
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    return sf::Vector2f(m_state.x, m_state.y);
  }

  RobotState getState() const {
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    return m_state;
  }

  void setState(RobotState state) {
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    m_state = state;
  }

  void resetState() {
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    m_state = RobotState();
    m_state.x = 96;
    m_state.y = 96;
    m_state.theta = 90.0f;
  }

  void setPosition(float x, float y) {
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    m_state.x = x;
    m_state.y = y;
  }

  void setOrientation(float theta) {
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    m_state.theta = theta;  //
  }

  float getOrientation() const {
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    return m_state.theta;
  }

  ///////////// Sensors
  void setSensorCallback(SensorDataCallback callback) {
    m_sensor_callback = callback;  //
  }

  SensorData& getSensorData() {
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    return m_sensor_data;
  }

  /// This is the primary way to get the robot to move
  void setSpeeds(float velocity, float omega) {
    CRITICAL_SECTION(g_robot_mutex) {
      m_state.velocity = velocity;
      m_state.omega = omega;
    }
  }

  [[nodiscard]] bool isRunning() const {
    return m_running.load();
  }

  [[nodiscard]] uint32_t millis() const {
    return m_ticks.load();
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
    if (!isRunning()) {
      return;
    }
    // The mutex will lock out the main thread while this block runs.
    CRITICAL_SECTION(g_robot_mutex) {
      m_ticks++;
      m_state.timestamp = m_ticks;

      // Ask the Application for a sensor update
      if (m_sensor_callback) {
        m_sensor_data = m_sensor_callback(m_state.x, m_state.y, m_state.theta);
      }

      // update the speeds
      m_state.velocity = std::clamp(m_state.velocity, -m_vMax, m_vMax);
      m_state.omega = std::clamp(m_state.omega, -m_omegaMax, +m_omegaMax);

      // accumulate distances
      float deltaDistance = m_state.velocity * deltaTime;
      m_state.total_distance += deltaDistance;
      m_state.cell_offset += deltaDistance;

      // wrap the angle
      m_state.theta += m_state.omega * deltaTime;
      if (m_state.theta >= 360.0f) {
        m_state.theta -= 360.0f;
      } else if (m_state.theta < 0.0f) {
        m_state.theta += 360.0f;
      }

      // calculate new location
      m_state.x += m_state.velocity * std::cos(m_state.theta * RADIANS) * deltaTime;
      m_state.y += m_state.velocity * std::sin(m_state.theta * RADIANS) * deltaTime;
    }
  }

 private:
  std::atomic<uint32_t> m_ticks;
  std::atomic<bool> m_running;

  SensorDataCallback m_sensor_callback = nullptr;

  SensorData m_sensor_data;  // Current sensor readings
  RobotState m_state;
  float m_vMax;
  float m_omegaMax;
};

#endif  // ROBOT_H
