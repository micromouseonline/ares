//
// Created by peter on 22/11/24.
//

#ifndef ROBOT_H
#define ROBOT_H

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include "common/core.h"
#include "robot-state.h"
#include "sensor-data.h"

/***
 * TODO Not sure this is the best place for this macro
 *
 * the macro looks odd because it uses a C++17 feature where
 * the if statement condition can include an initialiser.
 * This feature allows you to initialize variables within the
 * if statement, followed by a condition that determines if
 * the block should be executed. It's a neat way to declare
 * and initialise variables that are only used within the
 * scope of the if statement. Use it like this:
 *
 * if (initialiser; condition){
 *   // code that will run if condition is true
 * }
 *
 * The scope of any variables declared in the initializer
 * of an if statement is limited to the entire if block,
 * including both the condition and the code within the
 * braces {} of the if statement.
 *
 * Use the macro like this
 *
 * CRITICAL_SECTION(m_systick_mutex){
 *   // guarded code
 * }
 *
 */

#define CRITICAL_SECTION(the_mutex) if (std::lock_guard<std::mutex> lock(the_mutex); true)

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

  ~Robot() { Stop(); }

  void Start() {
    if (!m_running) {
      m_ticks = 0;
      m_running = true;
    }
  }

  // Stops the robot's thread
  void Stop() {
    if (m_running) {
      m_running = false;
    }
  }
  void Resume() { m_running = true; }

  ////// Accessors
  sf::Vector2f getPose() const {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return sf::Vector2f(m_state.x, m_state.y);
  }

  RobotState getState() const {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return m_state;
  }

  RobotState setState() const {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return m_state;
  }

  float getOrientation() const {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return m_state.theta;
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

  ///////////// Sensors
  void setSensorCallback(SensorDataCallback callback) {
    m_sensor_callback = callback;  //
  }

  SensorData& getSensorData() {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return m_sensor_data;
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
      if (m_sensor_callback) {
        m_sensor_data = m_sensor_callback(m_state.x, m_state.y, m_state.theta);
      }
      // updateMotionControllers();
      m_state.theta += m_state.omega * deltaTime;
      if (m_state.theta >= 360.0f) {
        m_state.theta -= 360.0f;
      } else if (m_state.theta < 0.0f) {
        m_state.theta += 360.0f;
      }
      m_state.x += m_state.v * std::cos(m_state.theta * RADIANS) * deltaTime;
      m_state.y += m_state.v * std::sin(m_state.theta * RADIANS) * deltaTime;
    }
  }

  uint32_t millis() {
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    return m_ticks;
  }

 private:
  uint32_t m_ticks;
  std::atomic<bool> m_running;  // Thread control flag
  SensorDataCallback m_sensor_callback = nullptr;
  mutable std::mutex m_robot_mutex;  // Protects access to m_pose and m_orientation
  SensorData m_sensor_data;          // Current sensor readings
  RobotState m_state;
};

#endif  // ROBOT_H
