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
#include "behaviour/behaviour.h"
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

  // Starts the robot in its own thread

  /***
   * The main application should call the Start() method to start the robot, passing
   * in a pointer to itself so that callbacks can be used. Fo example, the robot
   * can request sensor data from the application.
   *
   * Once started, the robot will run until Stop() is called.
   *
   * This robot instance will also start its own thread that repeatedly calls the
   * systick() method. In this case, systick runs at about 1kHz. Actually a little
   * less because we run the code then sleep for 1ms. Although the timing will not
   * be completely accurate, it should suffice for simulation.
   *
   * We could
   *
   * @param app - the calling thread.
   */
  void Start() {
    if (!m_running) {
      m_ticks = 0;
      m_running = true;
      m_thread = std::thread(&Robot::Run, this);
      m_systick_thread = std::thread(&Robot::systick, this);
    }
  }

  // Stops the robot's thread
  void Stop() {
    if (m_running) {
      m_running = false;
      if (m_systick_thread.joinable()) {
        m_systick_thread.join();
      }
      if (m_thread.joinable()) {
        m_thread.join();
      }
    }
  }

  ////// Accessors
  sf::Vector2f getPose() const {
    std::lock_guard<std::mutex> lock(m_systickMutex);
    return sf::Vector2f(m_state.x, m_state.y);
  }

  RobotState getState() const {
    std::lock_guard<std::mutex> lock(m_systickMutex);
    return m_state;
  }

  RobotState setState() const {
    std::lock_guard<std::mutex> lock(m_systickMutex);
    return m_state;
  }

  float getOrientation() const {
    std::lock_guard<std::mutex> lock(m_systickMutex);
    return m_state.theta;
  }

  void setPosition(float x, float y) {
    std::lock_guard<std::mutex> lock(m_systickMutex);
    m_state.x = x;
    m_state.y = y;
  }

  void setOrientation(float theta) {
    std::lock_guard<std::mutex> lock(m_systickMutex);
    m_state.theta = theta;  //
  }

  void setVelocity(float velocity) {
    std::lock_guard<std::mutex> lock(m_systickMutex);
    m_state.v = velocity;  //
  }

  void setOmega(float omega) {
    std::lock_guard<std::mutex> lock(m_systickMutex);
    m_state.omega = omega;  //
  }

  ///////////// Sensors
  void setSensorCallback(SensorDataCallback callback) {
    m_sensorCallback = callback;  //
  }

  SensorData& getSensorData() {
    std::lock_guard<std::mutex> lock(m_systickMutex);
    return m_sensorData;
  }

  /***
   * The systick() method runs in its own thread, with the loop running at, in
   * this case, about 1kHz. The systick() method simulates an interrupt triggered
   * by a timer on the hardware.
   *
   * In the real hardware, operations during the Interrupt Service Routine (ISR) are
   * mutually exclusive with the main code. This would not normally be the case here,
   * so everything that happens in systick() is guarded with a mutex. This ensures
   * the code in systick() can run and modify state as needed, while the top-level
   * robot code will block as soon as it tries to access any of the resources guarded
   * by the mutex, thus simulating the behavior of the hardware interrupt.
   *
   * If the top-level code needs to guarantee that systick() will not change any
   * shared state, it can also lock the same mutex for the duration of a critical
   * section. If the mutex is already locked, the top-level code will block until
   * systick() has finished that iteration. Then the top-level code will acquire the
   * mutex and block systick() until it is released.
   *
   * Care must be taken to ensure that these mutually cooperative functions do
   * not end up in a deadlock.
   */

  void systick() {
    using namespace std::chrono;
    /// NOTE: this runs a little fast on linux
    auto interval_us = duration_cast<microseconds>(duration<float>(m_loopTime));
    auto next_time = steady_clock::now() + interval_us;
    while (m_running) {
      // The mutex will lock out the main thread while this block runs.
      CRITICAL_SECTION(m_systickMutex) {
        m_ticks++;
        if (m_sensorCallback) {
          m_sensorData = m_sensorCallback();
        }
        // updateMotionControllers();
        m_state.theta += m_state.omega * m_loopTime;
        if (m_state.theta >= 360.0f) {
          m_state.theta -= 360.0f;
        } else if (m_state.theta < 0.0f) {
          m_state.theta += 360.0f;
        }
        m_state.x += m_state.v * std::cos(m_state.theta * RADIANS) * m_loopTime;
        m_state.y += m_state.v * std::sin(m_state.theta * RADIANS) * m_loopTime;
      }
      /// TODO: switching to an asynchronous method in Behaviour would call
      ///       systick directly from delays through the yield function??
      std::this_thread::sleep_until(next_time);
      next_time += interval_us;
    }
  }

  uint32_t millis() {
    std::lock_guard<std::mutex> lock(m_systickMutex);
    return m_ticks;
  }

 private:
  /***
   * The run() method is what gets called from the thread initialisation
   * It should run continuously when started. An atomic flag, m_running,
   * is used so that the parent thread can signal an orderly termination.
   */
  void Run() {
    while (m_running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100 Hz loop
    }
  }

  uint32_t m_ticks;
  std::thread m_systick_thread;
  std::thread m_thread;
  std::atomic<bool> m_running;  // Thread control flag
  float m_loopTime = 0.001f;

  SensorDataCallback m_sensorCallback = nullptr;
  mutable std::mutex m_systickMutex;  // Protects access to m_pose and m_orientation
  SensorData m_sensorData;            // Current sensor readings
  RobotState m_state;
};

#endif  // ROBOT_H
